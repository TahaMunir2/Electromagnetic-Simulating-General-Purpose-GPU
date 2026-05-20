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

Configure the blocks below — no other changes needed:
  • SOURCE_*    — driven conductor geometry (point / line / area)
  • GROUND_*    — optional grounded conductor
  • BOUNDARY_*  — outer absorbing PML or grounded enclosure
  • V_func      — source waveform (sine, Gaussian pulse, ramp-sine, …)
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
C_SPEED  = 1.0        # Effective wave speed (normalised)
COURANT  = 0.5        # S = c*dt/dx  (CFL safety factor, < 1/sqrt(2))
DT       = COURANT * DELTA / C_SPEED   # Time step (derived, do not edit)

# Coefficients for the Yee update equations (derived, do not edit):
#   CB = dt/dx   — how much curl-E drives dBz/dt
#   CE = c^2 * dt/dx — how much curl-Bz drives dE/dt
CB = DT / DELTA                        # Bz update coefficient
CE = C_SPEED**2 * DT / DELTA          # Ex, Ey update coefficient

# ── Animation ─────────────────────────────────────────────────────────────────
N_STEPS_PER_FRAME = 4     # FDTD steps computed between animation frames
                           # Higher = faster simulation, less smooth animation
N_FRAMES          = 300    # Total animation frames
INTERVAL          = 40     # ms between frames (~25 fps)

# ── Source waveform ────────────────────────────────────────────────────────────
VF_AMP   = 1.0        # Peak source amplitude
FREQ     = 0.05       # Normalised frequency  (wavelength = C_SPEED/FREQ = 20 cells)
                       # Keep FREQ < 0.1 to avoid grid dispersion

def V_func(t: float) -> float:
    """Source waveform as a function of time step t (in dt units)."""
    # Ramp-up sine — avoids startup transient artefacts
    tau = 1.0 / FREQ          # one period
    ramp = 1.0 - np.exp(-t / tau)
    return VF_AMP * ramp * np.sin(2 * np.pi * FREQ * t * DT)

# Alternatives (uncomment one):
# def V_func(t): return VF_AMP * np.sin(2*np.pi*FREQ*t*DT)        # pure sine
# def V_func(t):                                                    # Gaussian pulse
#     t0, sigma = 30, 10
#     return VF_AMP * np.exp(-0.5*((t-t0)/sigma)**2) * np.sin(2*np.pi*FREQ*t*DT)

# ── Source conductor ──────────────────────────────────────────────────────────
#  SOURCE_TYPE   "point" | "line" | "area"
#  SOURCE_X/Y    Centre position in grid units (0 = domain centre, range -SPAN..SPAN)
#  SOURCE_LEN    Length along primary axis [grid units]  (line / area)
#  SOURCE_WID    Width along secondary axis [grid units] (area only)
#  SOURCE_ORIENT "h" (horizontal) | "v" (vertical)       (line only)
#  SOURCE_FIELD  "Ex" | "Ey"  — which E component is driven (hard source)

SOURCE_TYPE   = "line"
SOURCE_X      = -15
SOURCE_Y      = 0
SOURCE_LEN    = 20
SOURCE_WID    = 4
SOURCE_ORIENT = "v"
SOURCE_FIELD  = "Ex"       # drive Ex component (or "Ey")

# ── Optional second source (set SOURCE2_ENABLE = True for interference) ────────
SOURCE2_ENABLE = False
SOURCE2_TYPE   = "point"
SOURCE2_X      = +15
SOURCE2_Y      = 0
SOURCE2_LEN    = 20
SOURCE2_WID    = 4
SOURCE2_ORIENT = "v"
SOURCE2_FIELD  = "Ex"
SOURCE2_PHASE  = np.pi     # phase offset relative to source 1 (radians)

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
    sigma = np.zeros((IMAX, IMAX), dtype=float)

    for i in range(IMAX):
        for edge_dist_fn, indices in [
            (lambda i: w - i,          range(w)),           # left
            (lambda i: i - (IMAX-1-w), range(IMAX-w, IMAX)), # right
        ]:
            if i in indices:
                d = max(0, w - i) if i < w else i - (IMAX - 1 - w)
                s = PML_SIGMA_MAX * (d / w) ** 3
                sigma[i, :] = np.maximum(sigma[i, :], s)
                sigma[:, i] = np.maximum(sigma[:, i], s)

    # Vectorised version (faster):
    sigma = np.zeros((IMAX, IMAX), dtype=float)
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


# Build masks
src1_mask  = _source_mask(SOURCE_TYPE,   SOURCE_X,  SOURCE_Y,
                           SOURCE_LEN,  SOURCE_WID, SOURCE_ORIENT)
src2_mask  = (_source_mask(SOURCE2_TYPE, SOURCE2_X, SOURCE2_Y,
                            SOURCE2_LEN, SOURCE2_WID, SOURCE2_ORIENT)
              if SOURCE2_ENABLE else np.zeros((IMAX, IMAX), dtype=bool))
gnd_mask   = (_source_mask(GROUND_TYPE,  GROUND_X,  GROUND_Y,
                            GROUND_LEN,  GROUND_WID, GROUND_ORIENT)
              if GROUND_EXIST else np.zeros((IMAX, IMAX), dtype=bool))

# Combined fixed-node mask (hard sources + ground)
# Cells in this mask are overwritten after each FDTD update (hard source).
all_fixed  = src1_mask | src2_mask | gnd_mask

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
    dEy_dx = (np.roll(Ey, -1, axis=1) - Ey) / DELTA
    dEx_dy = (np.roll(Ex, -1, axis=0) - Ex) / DELTA
    curl_E = dEy_dx - dEx_dy

    Bz = decay_Bz * Bz - CB * curl_E

    # ── 2. Update Ex from curl of Bz (Ampere, y-component) ───────────────────
    # dEx/dt = c^2 * dBz/dy
    # Backward difference:
    #   dBz/dy[i,j] = ( Bz[i, j] - Bz[i-1, j] ) / Delta
    dBz_dy = (Bz - np.roll(Bz, +1, axis=0)) / DELTA
    Ex = decay_E * Ex + CE * dBz_dy

    # ── 3. Update Ey from curl of Bz (Ampere, x-component) ───────────────────
    # dEy/dt = -c^2 * dBz/dx
    # Backward difference:
    #   dBz/dx[i,j] = ( Bz[i, j] - Bz[i, j-1] ) / Delta
    dBz_dx = (Bz - np.roll(Bz, +1, axis=1)) / DELTA
    Ey = decay_E * Ey - CE * dBz_dx

    # ── 4. PEC boundary: force E tangential = 0 on walls ─────────────────────
    if BOUNDARY_TYPE == "pec":
        Ex[pec_mask] = 0.0
        Ey[pec_mask] = 0.0
        Bz[pec_mask] = 0.0

    # ── 5. Hard-source injection ──────────────────────────────────────────────
    # Overwrite fixed-node cells with prescribed waveform (hard source).
    # Phase offset between source 1 and source 2 is applied here.
    v1 = V_func(n)
    if SOURCE_FIELD == "Ex":
        Ex[src1_mask] = v1
    else:
        Ey[src1_mask] = v1

    if SOURCE2_ENABLE:
        # Compute source 2 waveform with its own phase offset
        tau2  = 1.0 / FREQ
        ramp2 = 1.0 - np.exp(-n / tau2)
        v2    = VF_AMP * ramp2 * np.sin(2 * np.pi * FREQ * n * DT + SOURCE2_PHASE)
        if SOURCE2_FIELD == "Ex":
            Ex[src2_mask] = v2
        else:
            Ey[src2_mask] = v2

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
        Smag = np.abs(Ex * Bz) + np.abs(Ey * Bz)
        im = ax.imshow(Smag, origin='lower', extent=[-SPAN,SPAN,-SPAN,SPAN],
                       vmin=0, vmax=clim_e*clim_b, cmap='viridis', aspect='equal',
                       interpolation='bilinear')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04).ax.yaxis.set_tick_params(color='#8b949e', labelcolor='#8b949e')

    # Mark source positions on every panel
    if np.any(src1_mask):
        ys, xs = np.where(src1_mask)
        ax.plot(xs - SPAN, ys - SPAN, 'w+', ms=8, mew=1.5, alpha=0.8)
    if SOURCE2_ENABLE and np.any(src2_mask):
        ys, xs = np.where(src2_mask)
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
