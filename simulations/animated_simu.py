"""
General quasi-static AC electric field simulator.

Solves Laplace's equation on a 2-D grid using SOR, then animates the result
as the voltage source varies in time.  Because Laplace's equation is linear,
we solve once (unit voltage) and scale by V(t) at every animation frame.

Configure the blocks below — no other changes needed:
  • V_func      — voltage waveform (sine, square, sawtooth, …)
  • SOURCE_*    — geometry of the driven conductor (point / line / area)
  • BOUNDARY_*  — outer grounded enclosure (square box / cylinder / none)
  • EP_FUNC     — optional spatially-varying permittivity
"""

# 19 May 2026: This file has been verified up to line 282.
# Code after line 282 has not been fully checked yet and may contain bugs.
# Further testing/review is required.
# Will be working on UI for convenient input altering.

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

# ── Voltage waveform ──────────────────────────────────────────────────────────
VF_AMP   = 10.0     # Peak voltage amplitude (V)
FREQ     = 1.0      # Frequency (Hz)
N_FRAMES = 60       # Animation frames per full cycle
INTERVAL = 50       # Milliseconds between frames  (≈ 20 fps)

def V_func(t: float) -> float:
    """Instantaneous source voltage.  Replace with any function of t (seconds)."""
    return VF_AMP * np.sin(2 * np.pi * FREQ * t)
# Alternatives (uncomment one):
# def V_func(t): return VF_AMP * np.sign(np.sin(2*np.pi*FREQ*t))           # square wave
# def V_func(t): return VF_AMP * (2*(t*FREQ % 1) - 1)                      # sawtooth
# def V_func(t): return VF_AMP * np.sin(2*np.pi*FREQ*t)*np.exp(-3*FREQ*t)  # damped sine

# ── Source conductor ──────────────────────────────────────────────────────────
#
#  SOURCE_TYPE   "point"  — single grid cell  (a wire cross-section)
#                "line"   — straight conductor segment
#                "area"   — solid rectangular conductor (e.g. a flat plate)
#
#  SOURCE_X/Y    Centre position in grid units.  0 = domain centre.
#                Range: roughly −SPAN … +SPAN  (default SPAN = 50).
#
#  SOURCE_LEN    Extent of the conductor along its primary axis [grid units].
#                • "line" (h): full length along x
#                • "line" (v): full length along y
#                • "area":     full width  along x
#                Ignored for "point".
#
#  SOURCE_WID    Extent along the secondary axis [grid units].
#                • "area" only: full height along y.
#                Ignored for "point" and "line".
#
#  SOURCE_ORIENT "h" = horizontal, "v" = vertical  (only for SOURCE_TYPE = "line")
#
#  SOURCE_V      Potential of the conductor, unit-normalised.
#                The actual voltage at time t is  SOURCE_V * V_func(t).

SOURCE_TYPE   = "point"   # "point" | "line" | "area"
SOURCE_X      = 0        # Centre x (grid units)
SOURCE_Y      = 0         # Centre y (grid units)
SOURCE_LEN    = 20        # Conductor length / x-width  [grid units]  (area only)
SOURCE_WID    = 10        # Conductor height / y-width  [grid units]  (area only)
SOURCE_ORIENT = "h"       # "h" or "v"  (line only)
SOURCE_V      = +1.0      # Unit-normalised potential of the conductor

GROUND_EXIST = 0 # 1 exist, 0 not exist
GROUD_TYPE    = "point"   # "point" | "line" | "area"
GROUND_X = 20 # Centre x (grid units)
GROUND_Y = 20 # Centre y (grid units)
GROUND_LEN = 20        # Conductor length / x-width  [grid units]  (area only)
GROUND_WID = 10        # Conductor height / y-width  [grid units]  (area only)
GROUND_ORIENT = "h"       # "h" or "v"  (line only)

# ── Outer boundary (grounded enclosure) ───────────────────────────────────────
#
#  BOUNDARY_TYPE  "square"  — grounded rectangular box
#                 "circle"  — grounded cylindrical shell
#                 "none"    — no fixed outer boundary (open / periodic domain)
#
#  BOUNDARY_SIZE  "square": half-side length [grid units]  (≤ SPAN = 50)
#                 "circle": cylinder radius  [grid units]  (≤ SPAN = 50)
#                 Ignored for "none".
#
#  BOUNDARY_V     Potential of the enclosure, unit-normalised.

BOUNDARY_TYPE = "square"  # "square" | "circle" | "none"
BOUNDARY_SIZE = 45        # Half-side or radius [grid units]
BOUNDARY_V    = 0.0       # Unit-normalised boundary potential

# ── Permittivity distribution (optional) ─────────────────────────────────────
#
#  None  → uniform εᵣ = 1 everywhere.
#  Provide a callable f(X, Y) → array of shape (IMAX, IMAX).
#  X, Y are coordinate meshgrids in grid units (0 = domain centre).
#
#  Example — dielectric cylinder of radius 15, εᵣ = 2:
#  EP_FUNC = lambda X, Y: np.where(X**2 + Y**2 < 15**2, 2.0, 1.0)
#  EP_FUNC = None (uniform conductive board)

# ── Solver parameters ─────────────────────────────────────────────────────────
ALPHA    = 1.0      # Relaxation factor — must be 1.0 for vectorized (Jacobi-style) updates;
                    # ALPHA > 1 causes divergence because all cells are updated simultaneously.
IMAX     = 101      # Grid points per axis  (odd keeps a clean centre cell)
MAX_ITER = 15_000   # Safety cap on iterations
DELTA    = 1e-3     # Physical grid spacing (m) — used for E-field units (V/m)
CB       = 1.0 / (N_FRAMES * FREQ * DELTA)  # FDTD Faraday coeff = dt/DELTA  (s/m)

# ═══════════════════════════════════════════════════════════════════════════════
#  GRID  (derived from IMAX — do not edit)
# ═══════════════════════════════════════════════════════════════════════════════

SPAN = (IMAX - 1) // 2          # 50  — grid half-width in grid units
CEN  = SPAN                      # 0-indexed row/column of the centre cell

coords = np.arange(-SPAN, SPAN + 1, dtype=float)
X, Y   = np.meshgrid(coords, coords)   # X varies along columns, Y along rows

# ═══════════════════════════════════════════════════════════════════════════════
#  GEOMETRY BUILDER
# ═══════════════════════════════════════════════════════════════════════════════

def _ci(v: float) -> int:
    """Grid-unit coordinate → 0-based array index, clamped to [0, IMAX-1]."""
    return int(np.clip(round(float(v) + SPAN), 0, IMAX - 1))


def build_geometry():
    """
    Construct V0 and F_fix from the user configuration.

    Returns
    -------
    V0    : ndarray (IMAX, IMAX) — Dirichlet values for unit voltage.
    F_fix : ndarray (IMAX, IMAX) — 1 = free to iterate, 0 = fixed.
    EP    : ndarray (IMAX, IMAX) — relative permittivity.
    """
    F_fix = np.ones((IMAX, IMAX), dtype=float)
    # F_fix = 0 means the point is source or ground, not to iterate the value
    V0    = np.zeros((IMAX, IMAX), dtype=float)

    # ── source ────────────────────────────────────────────────────────────────
    cx   = _ci(SOURCE_X)
    cy   = _ci(SOURCE_Y)
    hl   = int(round(SOURCE_LEN / 2))
    hw   = int(round(SOURCE_WID / 2))

    if SOURCE_TYPE == "point":
        V0[cy, cx]    = SOURCE_V
        F_fix[cy, cx] = 0

    elif SOURCE_TYPE == "line":
        if SOURCE_ORIENT == "h":
            c0 = max(0, cx - hl);  c1 = min(IMAX, cx + hl + 1)
            V0[cy, c0:c1]    = SOURCE_V
            F_fix[cy, c0:c1] = 0
        else:                          # "v"
            r0 = max(0, cy - hl);  r1 = min(IMAX, cy + hl + 1)
            V0[r0:r1, cx]    = SOURCE_V
            F_fix[r0:r1, cx] = 0

    elif SOURCE_TYPE == "area":
        c0 = max(0, cx - hl);  c1 = min(IMAX, cx + hl + 1)
        r0 = max(0, cy - hw);  r1 = min(IMAX, cy + hw + 1)
        V0[r0:r1, c0:c1]    = SOURCE_V
        F_fix[r0:r1, c0:c1] = 0

    else:
        raise ValueError(
            f"SOURCE_TYPE must be 'point', 'line', or 'area'; got '{SOURCE_TYPE}'"
        )

    # ── ground ────────────────────────────────────────────────────────────────
    if GROUND_EXIST:
        gx   = _ci(GROUND_X)
        gy   = _ci(GROUND_Y)
        ghl  = int(round(GROUND_LEN / 2))
        ghw  = int(round(GROUND_WID / 2))

        if GROUD_TYPE == "point":
            V0[gy, gx]    = 0.0
            F_fix[gy, gx] = 0

        elif GROUD_TYPE == "line":
            if GROUND_ORIENT == "h":
                c0 = max(0, gx - ghl);  c1 = min(IMAX, gx + ghl + 1)
                V0[gy, c0:c1]    = 0.0
                F_fix[gy, c0:c1] = 0
            else:                          # "v"
                r0 = max(0, gy - ghl);  r1 = min(IMAX, gy + ghl + 1)
                V0[r0:r1, gx]    = 0.0
                F_fix[r0:r1, gx] = 0

        elif GROUD_TYPE == "area":
            c0 = max(0, gx - ghl);  c1 = min(IMAX, gx + ghl + 1)
            r0 = max(0, gy - ghw);  r1 = min(IMAX, gy + ghw + 1)
            V0[r0:r1, c0:c1]    = 0.0
            F_fix[r0:r1, c0:c1] = 0

        else:
            raise ValueError(
                f"GROUD_TYPE must be 'point', 'line', or 'area'; got '{GROUD_TYPE}'"
            )

    # ── outer boundary ────────────────────────────────────────────────────────
    bs = int(round(BOUNDARY_SIZE))

    if BOUNDARY_TYPE == "square":
        r0 = _ci(-bs);  r1 = _ci(+bs)
        c0 = _ci(-bs);  c1 = _ci(+bs)
        # top and bottom walls
        V0[r0, c0:c1+1]    = BOUNDARY_V;  F_fix[r0, c0:c1+1] = 0
        V0[r1, c0:c1+1]    = BOUNDARY_V;  F_fix[r1, c0:c1+1] = 0
        # left and right walls
        V0[r0:r1+1, c0]    = BOUNDARY_V;  F_fix[r0:r1+1, c0] = 0
        V0[r0:r1+1, c1]    = BOUNDARY_V;  F_fix[r0:r1+1, c1] = 0

    elif BOUNDARY_TYPE == "circle":
        mask = X**2 + Y**2 >= bs**2
        V0[mask]    = BOUNDARY_V
        F_fix[mask] = 0

    elif BOUNDARY_TYPE == "none":
        pass

    else:
        raise ValueError(
            f"BOUNDARY_TYPE must be 'square', 'circle', or 'none'; got '{BOUNDARY_TYPE}'"
        )

    # ── permittivity ──────────────────────────────────────────────────────────
    # if EP_FUNC is None:
    #     EP = np.ones((IMAX, IMAX), dtype=float)
    # else:
    #     EP = np.asarray(EP_FUNC(X, Y), dtype=float)
    #     if EP.shape != (IMAX, IMAX):
    #         raise ValueError(
    #             f"EP_FUNC must return shape ({IMAX}, {IMAX}); got {EP.shape}"
    #         )

    return V0, F_fix #, EP

V0, F_fix = build_geometry() # add EP if needed

# Spatial gradients of ε (constant in time — computed once)
# DEPDX = (np.roll(EP, 1, axis=1) - np.roll(EP, -1, axis=1)) / 2.0
# DEPDY = (np.roll(EP, 1, axis=0) - np.roll(EP, -1, axis=0)) / 2.0

# ═══════════════════════════════════════════════════════════════════════════════
#  POTENTIAL SOLVER  (unit-normalised voltage)
# ═══════════════════════════════════════════════════════════════════════════════

desc = f"source={SOURCE_TYPE}, boundary={BOUNDARY_TYPE}, α={ALPHA}"
print(f"Calculating potential by fixed iteration ({desc}) ...")
V = V0.copy()

for k in range(MAX_ITER):
    CX       = (np.roll(V,  1, axis=1) + np.roll(V, -1, axis=1)) / 4.0
    CY       = (np.roll(V,  1, axis=0) + np.roll(V, -1, axis=0)) / 4.0
    correction = CX + CY - V
    V         += ALPHA * correction * F_fix

    if k % 500 == 0:
        print(f"  iter {k:6d}")

print(f"Finished {MAX_ITER} potential iterations.")

# Base solution for a 1 V waveform value. At each time step:
#   phi(x,y,t) = V_base(x,y) * V_func(t)
#   E(x,y,t)   = E_base(x,y) * V_func(t)
# This works because Laplace's equation is linear.
EX_base   = (np.roll(V, 1, axis=1) - np.roll(V, -1, axis=1)) / (2.0 * DELTA)
EY_base   = (np.roll(V, 1, axis=0) - np.roll(V, -1, axis=0)) / (2.0 * DELTA)
EMOD_base = np.sqrt(EX_base**2 + EY_base**2)
V_base    = V.copy()

print("Solver complete.  Building animation…")

# ═══════════════════════════════════════════════════════════════════════════════
#  PRE-COMPUTE TIME-SERIES
# ═══════════════════════════════════════════════════════════════════════════════

# Use a half-frame offset so the sine waveform does not start exactly at 0 V.
# A 0 V frame has zero potential and zero electric field, so contour plots look blank.
times   = (np.arange(N_FRAMES) + 0.5) / (N_FRAMES * FREQ)
vf_vals = np.array([V_func(t) for t in times])

# Potential and electric field at every animation time step.
PHI_frames  = vf_vals[:, None, None] * V_base[None, :, :]
EX_frames   = vf_vals[:, None, None] * EX_base[None, :, :]
EY_frames   = vf_vals[:, None, None] * EY_base[None, :, :]
EMOD_frames = np.abs(vf_vals)[:, None, None] * EMOD_base[None, :, :]

# Magnetic field Bz via FDTD Faraday's law (z-component, pointing out of plane):
#   Bz_new = Bz_old - CB * [(Ex(i,j+1)-Ex(i,j)) - (Ey(i+1,j)-Ey(i,j))]
# Convention: i = x-index (axis=1 / columns), j = y-index (axis=0 / rows).
# So Ex(i,j+1)-Ex(i,j) is the forward y-difference of Ex  → roll axis=0 by -1.
#    Ey(i+1,j)-Ey(i,j) is the forward x-difference of Ey  → roll axis=1 by -1.
BZ_frames = np.zeros((N_FRAMES, IMAX, IMAX))
_Bz = np.zeros((IMAX, IMAX))
for _n in range(N_FRAMES):
    _dEx = np.roll(EX_frames[_n], -1, axis=0) - EX_frames[_n]  # Ex(i,j+1) - Ex(i,j)
    _dEy = np.roll(EY_frames[_n], -1, axis=1) - EY_frames[_n]  # Ey(i+1,j) - Ey(i,j)
    _Bz  = _Bz - CB * (_dEx - _dEy)
    BZ_frames[_n] = _Bz
# Normalise to [-1, 1]: the quasi-static Laplace field is irrotational (curl E = 0 in
# theory), so absolute Bz values are numerical artefacts from point-source singularities
# and finite-difference discretisation.  Only the spatial pattern is meaningful.
_bz_peak = float(np.max(np.abs(BZ_frames))) or 1.0
BZ_frames /= _bz_peak

# ═══════════════════════════════════════════════════════════════════════════════
#  FIGURE & AXES
# ═══════════════════════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(13, 10), facecolor='white')
gs  = GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.40)

ax_phi = fig.add_subplot(gs[0, 0])   # Potential contour map
ax_E   = fig.add_subplot(gs[0, 1])   # |E| filled + field lines
ax_Bz  = fig.add_subplot(gs[1, 0])   # Bz heatmap
ax_ts  = fig.add_subplot(gs[1, 1])   # V(t) time trace

LM       = 1
EMOD_max = float(np.max(EMOD_base[LM:IMAX-LM, LM:IMAX-LM])) * VF_AMP
EMOD_max = EMOD_max if EMOD_max > 0 else 1.0
LEV      = np.linspace(0, EMOD_max, 50)
PHI_LEV  = np.linspace(-VF_AMP, VF_AMP, 50)

BZ_abs_max = 1.0  # frames are already normalised to [-1, 1]

# ── Static panel: permittivity ────────────────────────────────────────────────
# im_ep = ax_ep.contourf(X, Y, EP, levels=20, cmap='jet', vmin=1, vmax=max(3, EP.max()))
# fig.colorbar(im_ep, ax=ax_ep, fraction=0.046, pad=0.04)
# ax_ep.set_title('Relative permittivity εᵣ', fontsize=11, fontweight='bold')
# ax_ep.set_xlabel('x (grid units)'); ax_ep.set_ylabel('y (grid units)')
# ax_ep.set_aspect('equal')
# ax_ep.set_xlim(-SPAN, SPAN); ax_ep.set_ylim(-SPAN, SPAN)

# ── Static panel: voltage time trace ─────────────────────────────────────────
ax_ts.plot(times * 1e3, vf_vals, 'k-', lw=1.5, alpha=0.6, label='V(t)')
marker_pt, = ax_ts.plot([], [], 'ro', ms=9, zorder=5)
ax_ts.axhline(0, color='gray', lw=0.8, ls='--')
ax_ts.set_xlabel('Time (ms)'); ax_ts.set_ylabel('V (V)')
ax_ts.set_title('Source voltage V(t)', fontsize=11, fontweight='bold')
ax_ts.legend(fontsize=9); ax_ts.grid(True, alpha=0.3)

# ── Initial frame ─────────────────────────────────────────────────────────────
_vf0    = vf_vals[0]
_Vs0    = PHI_frames[0]
_EXs0   = EX_frames[0]
_EYs0   = EY_frames[0]
_EMODs0 = EMOD_frames[0]

ax_phi.contourf(X, Y, _Vs0, PHI_LEV, cmap='jet')
ax_phi.set_title('Potential φ (V)', fontsize=11, fontweight='bold')
ax_phi.set_xlabel('x (grid units)'); ax_phi.set_ylabel('y (grid units)')
ax_phi.set_aspect('equal')
ax_phi.set_xlim(-SPAN, SPAN); ax_phi.set_ylim(-SPAN, SPAN)

ax_E.contourf(X, Y, _EMODs0, LEV, cmap='jet')
if abs(_vf0) > 1e-6 * VF_AMP:
    ax_E.streamplot(X, Y, _EXs0, _EYs0, color='white', linewidth=1.0, density=1.2)
ax_E.set_title('Electric field magnitude |E| (V/m)', fontsize=11, fontweight='bold')
ax_E.set_xlabel('x (grid units)'); ax_E.set_ylabel('y (grid units)')
ax_E.set_aspect('equal')
ax_E.set_xlim(-SPAN, SPAN); ax_E.set_ylim(-SPAN, SPAN)

im_Bz = ax_Bz.imshow(
    BZ_frames[0], origin='lower',
    extent=[-SPAN, SPAN, -SPAN, SPAN],
    vmin=-BZ_abs_max, vmax=BZ_abs_max,
    cmap='RdBu_r', aspect='equal',
)
fig.colorbar(im_Bz, ax=ax_Bz, fraction=0.046, pad=0.04, label='Bz (normalised)')
ax_Bz.set_title('Magnetic field Bz (spatial pattern, normalised)', fontsize=11, fontweight='bold')
ax_Bz.set_xlabel('x (grid units)'); ax_Bz.set_ylabel('y (grid units)')

suptitle = fig.suptitle('', fontsize=13, fontweight='bold', y=0.98)

# ═══════════════════════════════════════════════════════════════════════════════
#  ANIMATION
# ═══════════════════════════════════════════════════════════════════════════════

def update(frame_idx: int):
    t  = times[frame_idx]
    vf = float(vf_vals[frame_idx])

    Vs    = PHI_frames[frame_idx]
    EXs   = EX_frames[frame_idx]
    EYs   = EY_frames[frame_idx]
    EMODs = EMOD_frames[frame_idx]

    ax_phi.cla()
    ax_phi.contourf(X, Y, Vs, PHI_LEV, cmap='jet')
    ax_phi.set_title('Potential φ (V)', fontsize=11, fontweight='bold')
    ax_phi.set_xlabel('x (grid units)'); ax_phi.set_ylabel('y (grid units)')
    ax_phi.set_aspect('equal')
    ax_phi.set_xlim(-SPAN, SPAN); ax_phi.set_ylim(-SPAN, SPAN)

    ax_E.cla()
    ax_E.contourf(X, Y, EMODs, LEV, cmap='jet')
    if abs(vf) > 1e-6 * VF_AMP:
        ax_E.streamplot(X, Y, EXs, EYs, color='white', linewidth=1.0, density=1.2)
    ax_E.set_title('Electric field magnitude |E| (V/m)', fontsize=11, fontweight='bold')
    ax_E.set_xlabel('x (grid units)'); ax_E.set_ylabel('y (grid units)')
    ax_E.set_aspect('equal')
    ax_E.set_xlim(-SPAN, SPAN); ax_E.set_ylim(-SPAN, SPAN)

    im_Bz.set_data(BZ_frames[frame_idx])

    marker_pt.set_data([t * 1e3], [vf])

    src_info = f"{SOURCE_TYPE}"
    if SOURCE_TYPE == "line":
        src_info += f"({SOURCE_ORIENT}, len={SOURCE_LEN})"
    elif SOURCE_TYPE == "area":
        src_info += f"(len={SOURCE_LEN}, wid={SOURCE_WID})"
    suptitle.set_text(
        f"source={src_info}  boundary={BOUNDARY_TYPE}(size={BOUNDARY_SIZE})"
        f"  |  t={t*1e3:.1f} ms  V={vf:+.2f} V  (f={FREQ} Hz)"
    )
    return []


ani = animation.FuncAnimation(
    fig, update,
    frames=N_FRAMES,
    interval=INTERVAL,
    blit=False,
    repeat=True,
)

plt.show()
