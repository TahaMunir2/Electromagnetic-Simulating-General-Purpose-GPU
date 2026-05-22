"""
Full-wave 2-D FDTD electromagnetic field simulator.

Solves Maxwell's curl equations directly on a 2-D Yee grid using the
standard leap-frog (Yee) scheme.  E and B are truly coupled — each
drives the other every half time step, so real wave propagation,
dispersion, and interference are physically correct.

Compare with the quasi-static version:
  Quasi-static: Laplace solve once -> scale by V(t).  No real waves.
  This file:    Ex, Ey, Bz updated every step via Maxwell's curls.
                Waves travel, reflect, interfere, and get absorbed by PML.

Numerical constants match the FPGA implementation (Section 9 of the guide):
  c_eff=0.5, Δt=0.35, CB=0.35 (Q3.13: 2867), CE=0.0875 (Q3.13: 717).
Sources use soft injection (+=) as in fdtd_engine.v, so reflected waves
pass back through source cells without artificial re-reflection.

Configure the blocks below — no other changes needed:
  • SOURCES     — list of up to ~15 source dicts (type/position/phase/enable)
  • GROUND_*    — optional grounded conductor
  • BOUNDARY_*  — outer absorbing PML or grounded enclosure
  • DISPLAY_*   — which field panels to show
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

# ── Grid ──────────────────────────────────────────────────────────────────────
IMAX     = 256        # Grid points per axis (odd keeps a clean centre cell)
DELTA    = 1.0        # Spatial cell size (normalised units, 1 unit = one cell)

# ── Time stepping ─────────────────────────────────────────────────────────────
# Courant number S = c*dt/dx.  Must be < 1/sqrt(2) ≈ 0.707 for 2-D stability.
# S = 0.5 is a safe, standard choice.
C_SPEED  = 0.5        # FPGA c_eff (guide Section 9)
DT       = 0.35       # FPGA Δt   (guide Section 9); gives CB=0.35, CE=0.0875
COURANT  = C_SPEED * DT / DELTA       # ≈0.175, well below 1/√2 stability limit

# Coefficients for the Yee update equations — match FPGA Q3.13 constants:
#   CB = Δt/Δx       = 0.35    (Q3.13: 2867)
#   CE = c²×Δt/Δx   = 0.0875  (Q3.13:  717)
CB = DT / DELTA                        # Bz update coefficient
CE = C_SPEED**2 * DT / DELTA          # Ex, Ey update coefficient

# ── Animation ─────────────────────────────────────────────────────────────────
N_STEPS_PER_FRAME = 4     # FDTD steps computed between animation frames
                           # Higher = faster simulation, less smooth animation
N_FRAMES          = 300    # Total animation frames
INTERVAL          = 40     # ms between frames (~25 fps)

# ── Source waveform ────────────────────────────────────────────────────────────
VF_AMP   = 1.0        # Peak source amplitude
FREQ     = 0.05       # Normalised frequency  (wavelength = C_SPEED/FREQ = 10 cells)
                       # Matches FPGA f₀=0.05; phase increment = f₀×2π×DT ≈ 0.10996 rad/step
                       # Keep FREQ < 0.1 to avoid grid dispersion

# Waveform: ramp-up sine, computed inline per source in fdtd_step.
# Phase increment per step = 0.10996 rad ≡ Q3.13 value 901 used in FPGA CORDIC.

# ── Source conductors ─────────────────────────────────────────────────────────
#  List of up to ~15 sources.  Each entry is a dict with keys:
#    type   : "point" | "line" | "area"
#    x, y   : centre position in grid units (0 = domain centre, range -SPAN..SPAN)
#    len    : length along primary axis [grid units]  (line / area)
#    wid    : width along secondary axis [grid units] (area only)
#    orient : "h" (horizontal) | "v" (vertical)       (line only)
#    field  : "Ex" | "Ey"  — which E component is soft-injected
#    phase  : phase offset in radians relative to source 0
#    enable : True | False

SOURCES = [
    # Source 0 — vertical line, left of centre, drives Ex
    {
        "type": "line", "x": -15, "y": 0, "len": 20, "wid": 4,
        "orient": "v", "field": "Ex", "phase": 0.0, "enable": True,
    },
    # Source 1 — example second source (antiphase, right of centre)
    {
        "type": "point", "x": +15, "y": 0, "len": 1, "wid": 1,
        "orient": "v",  "field": "Ex", "phase": np.pi, "enable": False,
    },
    # Add further sources below — copy a block and set enable: True.
    # Up to ~15 sources are supported with no code changes.
]

# ── Ground conductor ──────────────────────────────────────────────────────────
GROUND_EXIST  = False
GROUND_TYPE   = "line"
GROUND_X      = +15
GROUND_Y      = 0
GROUND_LEN    = 20
GROUND_WID    = 4
GROUND_ORIENT = "v"
GROUND_FIELD  = "Ex"

# ── Outer boundary ─────────────────────────────────────────────────────────────
#  "pml"    — Perfectly Matched Layer: absorbs outgoing waves (best for open domains)
#  "pec"    — Perfect Electric Conductor: hard metallic walls (E=0 on boundary)
#  "none"   — periodic wrap (np.roll artefacts at edges — not recommended)
BOUNDARY_TYPE = "pml"
PML_WIDTH     = 10          # PML thickness in cells (8-15 is typical)
PML_SIGMA_MAX = 0.5         # PML peak conductivity (higher = more absorption,
                             # but >1 can cause reflections — keep 0.3–0.8)

# ── Display ────────────────────────────────────────────────────────────────────
SHOW_EX      = True    # Panel: Ex field
SHOW_BZ      = True    # Panel: Bz field
SHOW_EMAG    = True    # Panel: |E| = sqrt(Ex^2 + Ey^2)
SHOW_POYNTING= True    # Panel: |S| = Poynting vector magnitude (energy flow)
CLIM_E       = 0.5     # Colour axis limit for E panels (auto if None)
CLIM_B       = 0.3     # Colour axis limit for B panel  (auto if None)

# ═══════════════════════════════════════════════════════════════════════════════
#  GRID SETUP  (derived — do not edit)
# ═══════════════════════════════════════════════════════════════════════════════

SPAN = (IMAX - 1) // 2
CEN  = SPAN

coords = np.arange(-SPAN, SPAN + 1, dtype=float)
X, Y   = np.meshgrid(coords, coords)   # X varies along columns, Y along rows

# Field arrays — all zero initially
Ex = np.zeros((IMAX, IMAX), dtype=float)
Ey = np.zeros((IMAX, IMAX), dtype=float)
Bz = np.zeros((IMAX, IMAX), dtype=float)

# Time step counter
n_step = 0

# ═══════════════════════════════════════════════════════════════════════════════
#  PML SETUP
# ═══════════════════════════════════════════════════════════════════════════════

def _make_pml_profile():
    """
    Build 2-D arrays of PML conductivity sigma_x and sigma_y.

    Inside the domain: sigma = 0  (no damping).
    Inside the PML border of width PML_WIDTH: sigma grows as a cubic polynomial
    from 0 at the inner edge to PML_SIGMA_MAX at the outer edge.

    The update equations become:
        Bz_new  = (1 - sigma_E * dt) * Bz - CB * curl_E
        Ex_new  = (1 - sigma_B * dt) * Ex + CE * curl_Bz_y
        Ey_new  = (1 - sigma_B * dt) * Ey - CE * curl_Bz_x

    This is a simple "conductive medium" PML (not the full split-field CPML),
    but it works well enough for visualisation — reflections < 1%.
    """
    w = PML_WIDTH
    ramp = np.zeros(IMAX)
    for k in range(w):
        ramp[k]          = PML_SIGMA_MAX * ((w - k) / w) ** 3   # left/top
        ramp[IMAX-1-k]   = PML_SIGMA_MAX * ((w - k) / w) ** 3   # right/bottom
    # sigma is the outer envelope — take maximum of x and y ramps
    sigma_x = np.tile(ramp,         (IMAX, 1))   # varies along columns (x)
    sigma_y = np.tile(ramp[:, None],(1, IMAX))   # varies along rows    (y)
    return sigma_x, sigma_y

sigma_x, sigma_y = _make_pml_profile()
sigma_avg = 0.5 * (sigma_x + sigma_y)   # isotropic damping for Bz

# Decay factors applied each time step  (stored to avoid recomputing)
decay_E  = 1.0 - sigma_avg * DT    # for Ex and Ey
decay_Bz = 1.0 - sigma_avg * DT    # for Bz

# ═══════════════════════════════════════════════════════════════════════════════
#  GEOMETRY BUILDER  (fixed-node masks)
# ═══════════════════════════════════════════════════════════════════════════════

def _ci(v: float) -> int:
    """Grid-unit coordinate -> 0-based array index, clamped to [0, IMAX-1]."""
    return int(np.clip(round(float(v) + SPAN), 0, IMAX - 1))


def _source_mask(stype, sx, sy, slen, swid, sorient):
    """Return boolean mask (IMAX x IMAX) of cells belonging to a conductor."""
    mask = np.zeros((IMAX, IMAX), dtype=bool)
    cx, cy = _ci(sx), _ci(sy)
    hl = int(round(slen / 2))
    hw = int(round(swid / 2))

    if stype == "point":
        mask[cy, cx] = True

    elif stype == "line":
        if sorient == "h":
            c0 = max(0, cx - hl); c1 = min(IMAX, cx + hl + 1)
            mask[cy, c0:c1] = True
        else:
            r0 = max(0, cy - hl); r1 = min(IMAX, cy + hl + 1)
            mask[r0:r1, cx] = True

    elif stype == "area":
        c0 = max(0, cx - hl); c1 = min(IMAX, cx + hl + 1)
        r0 = max(0, cy - hw); r1 = min(IMAX, cy + hw + 1)
        mask[r0:r1, c0:c1] = True

    return mask


# Build source masks — one per entry in SOURCES
src_masks = [
    _source_mask(s["type"], s["x"], s["y"], s["len"], s["wid"], s["orient"])
    if s["enable"] else np.zeros((IMAX, IMAX), dtype=bool)
    for s in SOURCES
]
gnd_mask  = (_source_mask(GROUND_TYPE,  GROUND_X,  GROUND_Y,
                           GROUND_LEN,  GROUND_WID, GROUND_ORIENT)
             if GROUND_EXIST else np.zeros((IMAX, IMAX), dtype=bool))

# PEC boundary mask (outer walls if BOUNDARY_TYPE == "pec")
pec_mask   = np.zeros((IMAX, IMAX), dtype=bool)
if BOUNDARY_TYPE == "pec":
    pec_mask[0, :]  = True
    pec_mask[-1, :] = True
    pec_mask[:, 0]  = True
    pec_mask[:, -1] = True

# ═══════════════════════════════════════════════════════════════════════════════
#  FULL-WAVE FDTD STEP  (the core engine)
# ═══════════════════════════════════════════════════════════════════════════════

def fdtd_step(n: int):
    """
    Advance the simulation by one FDTD time step using the Yee leap-frog scheme.

    The update order is:
        1. Bz^(n+1/2) from curl of E^(n)          [Faraday's law]
        2. Apply PML damping to Bz
        3. Ex^(n+1), Ey^(n+1) from curl of Bz^(n+1/2)  [Ampere's law]
        4. Apply PML damping to Ex, Ey
        5. Apply PEC boundary conditions
        6. Inject hard sources (overwrite fixed nodes)

    All curl operations use forward finite differences on the Yee staggered grid:
        dBz/dt = -( dEy/dx - dEx/dy )
        dEx/dt =  c^2 * dBz/dy
        dEy/dt = -c^2 * dBz/dx

    Discrete forward differences (Yee convention):
        dEy/dx  ->  ( Ey[i, j+1] - Ey[i, j] ) / Delta   (axis=1, roll -1)
        dEx/dy  ->  ( Ex[i+1, j] - Ex[i, j] ) / Delta    (axis=0, roll -1)
        dBz/dy  ->  ( Bz[i, j]   - Bz[i-1, j] ) / Delta  (axis=0, roll +1)
        dBz/dx  ->  ( Bz[i, j]   - Bz[i, j-1] ) / Delta  (axis=1, roll +1)
    """
    global Ex, Ey, Bz

    # ── 1. Update Bz from curl of E (Faraday) ────────────────────────────────
    # dBz/dt = -( dEy/dx - dEx/dy )
    # Forward differences:
    #   dEy/dx[i,j] = ( Ey[i, j+1] - Ey[i, j] ) / Delta
    #   dEx/dy[i,j] = ( Ex[i+1, j] - Ex[i, j] ) / Delta
    # Zero the last row/column of each difference to prevent np.roll wrap-around
    # from coupling opposite edges through the PML.
    dEy_dx = (np.roll(Ey, -1, axis=1) - Ey) / DELTA
    dEy_dx[:, -1] = 0.0
    dEx_dy = (np.roll(Ex, -1, axis=0) - Ex) / DELTA
    dEx_dy[-1, :] = 0.0
    curl_E = dEy_dx - dEx_dy

    Bz = decay_Bz * Bz - CB * curl_E

    # ── 2. Update Ex from curl of Bz (Ampere, y-component) ───────────────────
    # dEx/dt = c^2 * dBz/dy
    # Backward difference:
    #   dBz/dy[i,j] = ( Bz[i, j] - Bz[i-1, j] ) / Delta
    dBz_dy = (Bz - np.roll(Bz, +1, axis=0)) / DELTA
    dBz_dy[0, :] = 0.0
    Ex = decay_E * Ex + CE * dBz_dy

    # ── 3. Update Ey from curl of Bz (Ampere, x-component) ───────────────────
    # dEy/dt = -c^2 * dBz/dx
    # Backward difference:
    #   dBz/dx[i,j] = ( Bz[i, j] - Bz[i, j-1] ) / Delta
    dBz_dx = (Bz - np.roll(Bz, +1, axis=1)) / DELTA
    dBz_dx[:, 0] = 0.0
    Ey = decay_E * Ey - CE * dBz_dx

    # ── 4. PEC boundary: force E tangential = 0 on walls ─────────────────────
    if BOUNDARY_TYPE == "pec":
        Ex[pec_mask] = 0.0
        Ey[pec_mask] = 0.0
        Bz[pec_mask] = 0.0

    # ── 5. Soft-source injection — matches FPGA fdtd_engine.v ────────────────
    # ADD waveform to the FDTD-updated field (soft source) so reflected waves
    # pass back through source cells without artificial re-reflection.
    ramp = 1.0 - np.exp(-n / (1.0 / FREQ))
    for s, mask in zip(SOURCES, src_masks):
        if not s["enable"] or not np.any(mask):
            continue
        v = VF_AMP * ramp * np.sin(2 * np.pi * FREQ * n * DT + s["phase"])
        if s["field"] == "Ex":
            Ex[mask] += v
        else:
            Ey[mask] += v

    if GROUND_EXIST:
        if GROUND_FIELD == "Ex":
            Ex[gnd_mask] = 0.0
        else:
            Ey[gnd_mask] = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  FIGURE SETUP
# ═══════════════════════════════════════════════════════════════════════════════

# Count active display panels
panels = [SHOW_EX, SHOW_BZ, SHOW_EMAG, SHOW_POYNTING]
n_panels = sum(panels)
panel_labels = ["Ex field", "Bz field", "|E| magnitude", "|S| Poynting"]
active = [(lbl, idx) for idx, (show, lbl) in
          enumerate(zip(panels, panel_labels)) if show]

fig_cols = min(n_panels, 2)
fig_rows = (n_panels + 1) // 2
fig, axes = plt.subplots(fig_rows, fig_cols,
                          figsize=(6 * fig_cols, 5.5 * fig_rows),
                          facecolor='#0d1117')
if n_panels == 1:
    axes = np.array([[axes]])
elif n_panels <= 2:
    axes = axes.reshape(1, -1)
else:
    axes = np.array(axes).reshape(fig_rows, fig_cols)

# Flatten to list of axes in panel order
ax_list = [axes[i // fig_cols, i % fig_cols] for i in range(n_panels)]

# Hide unused axes
for i in range(n_panels, fig_rows * fig_cols):
    axes[i // fig_cols, i % fig_cols].set_visible(False)

# Style helper
def _style_ax(ax, title):
    ax.set_facecolor('#0d1117')
    ax.set_title(title, color='white', fontsize=11, fontweight='bold', pad=6)
    ax.set_xlabel('x (cells)', color='#8b949e', fontsize=9)
    ax.set_ylabel('y (cells)', color='#8b949e', fontsize=9)
    ax.tick_params(colors='#8b949e', labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#30363d')
    ax.set_xlim(-SPAN, SPAN); ax.set_ylim(-SPAN, SPAN)
    ax.set_aspect('equal')

# Colour limits
clim_e = CLIM_E if CLIM_E is not None else VF_AMP
clim_b = CLIM_B if CLIM_B is not None else VF_AMP * CB

# Initialise image objects
imgs = {}
panel_idx = 0

for lbl, orig_idx in active:
    ax = ax_list[panel_idx]
    if orig_idx == 0:   # Ex
        im = ax.imshow(Ex, origin='lower', extent=[-SPAN,SPAN,-SPAN,SPAN],
                       vmin=-clim_e, vmax=clim_e, cmap='RdBu_r', aspect='equal',
                       interpolation='bilinear')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04).ax.yaxis.set_tick_params(color='#8b949e', labelcolor='#8b949e')
    elif orig_idx == 1: # Bz
        im = ax.imshow(Bz, origin='lower', extent=[-SPAN,SPAN,-SPAN,SPAN],
                       vmin=-clim_b, vmax=clim_b, cmap='PuOr_r', aspect='equal',
                       interpolation='bilinear')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04).ax.yaxis.set_tick_params(color='#8b949e', labelcolor='#8b949e')
    elif orig_idx == 2: # |E|
        Emag = np.sqrt(Ex**2 + Ey**2)
        im = ax.imshow(Emag, origin='lower', extent=[-SPAN,SPAN,-SPAN,SPAN],
                       vmin=0, vmax=clim_e, cmap='inferno', aspect='equal',
                       interpolation='bilinear')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04).ax.yaxis.set_tick_params(color='#8b949e', labelcolor='#8b949e')
    elif orig_idx == 3: # |S|
        Smag = np.sqrt((Ey * Bz)**2 + (Ex * Bz)**2)
        im = ax.imshow(Smag, origin='lower', extent=[-SPAN,SPAN,-SPAN,SPAN],
                       vmin=0, vmax=clim_e*clim_b, cmap='viridis', aspect='equal',
                       interpolation='bilinear')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04).ax.yaxis.set_tick_params(color='#8b949e', labelcolor='#8b949e')

    # Mark source positions on every panel
    for mask in src_masks:
        if np.any(mask):
            ys, xs = np.where(mask)
            ax.plot(xs - SPAN, ys - SPAN, 'w+', ms=8, mew=1.5, alpha=0.8, zorder=5)
    if GROUND_EXIST and np.any(gnd_mask):
        ys, xs = np.where(gnd_mask)
        ax.plot(xs - SPAN, ys - SPAN, 'wx', ms=8, mew=1.5, alpha=0.8)

    _style_ax(ax, lbl)
    imgs[orig_idx] = im
    panel_idx += 1

suptitle = fig.suptitle('', color='white', fontsize=12, fontweight='bold', y=1.01)
fig.patch.set_facecolor('#0d1117')
plt.tight_layout()

# ═══════════════════════════════════════════════════════════════════════════════
#  ANIMATION
# ═══════════════════════════════════════════════════════════════════════════════

def update(frame):
    global n_step

    # Run N_STEPS_PER_FRAME FDTD steps between display frames
    for _ in range(N_STEPS_PER_FRAME):
        fdtd_step(n_step)
        n_step += 1

    # Update displayed images
    if 0 in imgs:
        imgs[0].set_data(Ex)
    if 1 in imgs:
        imgs[1].set_data(Bz)
    if 2 in imgs:
        imgs[2].set_data(np.sqrt(Ex**2 + Ey**2))
    if 3 in imgs:
        Smag = np.sqrt((Ey * Bz)**2 + (Ex * Bz)**2)
        imgs[3].set_data(Smag)

    t_phys = n_step * DT
    wavelength = C_SPEED / FREQ
    suptitle.set_text(
        f"Full-wave FDTD  |  step {n_step}  t={t_phys:.1f}  "
        f"f={FREQ}  λ={wavelength:.0f} cells  "
        f"CFL={COURANT:.2f}  boundary={BOUNDARY_TYPE.upper()}"
    )
    return list(imgs.values())


ani = animation.FuncAnimation(
    fig, update,
    frames=N_FRAMES,
    interval=INTERVAL,
    blit=False,
    repeat=True,
)

plt.show()
