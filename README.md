# ElectroMagnetic FPGA 3D Renderer — FDTD Simulation

Python golden-reference for the FPGA full-wave electromagnetic field visualiser
(EE2 Design Project, Imperial College London).

---

## What it does

`fdtd_fullwave_simulation.py` simulates how electromagnetic waves propagate across
a 256 × 256 cell grid by solving Maxwell's curl equations directly at every time
step. Four live panels are animated:

| Panel | Field | Colour map |
|---|---|---|
| Ex field | Horizontal electric field component | Red–Blue |
| Bz field | Magnetic field (out of plane) | Purple–Orange |
| \|E\| magnitude | sqrt(Ex² + Ey²) | Inferno |
| \|S\| Poynting | Energy flow magnitude | Viridis |

White `+` markers show where each source is injecting. White `×` markers show
grounded conductor cells (if enabled).

---

## Physics — how the FDTD update works

The simulation uses the **Yee leap-frog scheme** on a 2-D TM-mode grid (Ex, Ey, Bz).
Each time step advances the fields in three stages:

```
1. Bz^(n+½)  ←  Bz^(n-½)  −  CB × curl(E^n)       [Faraday's law]
2. Ex^(n+1)  ←  Ex^(n)     +  CE × ∂Bz/∂y          [Ampere's law, x]
3. Ey^(n+1)  ←  Ey^(n)     −  CE × ∂Bz/∂x          [Ampere's law, y]
```

All spatial derivatives are first-order finite differences on the staggered Yee
grid. PML decay factors are applied to both fields every step to absorb outgoing
waves at the edges.

### Numerical constants — matched to the FPGA

| Constant | Value | FPGA Q3.13 | Meaning |
|---|---|---|---|
| c_eff | 0.5 | 4096 | Effective wave speed |
| Δt | 0.35 | 2867 | Time step |
| CB = Δt/Δx | 0.35 | 2867 | Bz update coefficient |
| CE = c²×Δt/Δx | 0.0875 | 717 | Ex, Ey update coefficient |
| f₀ | 0.05 | 410 | Normalised source frequency |
| λ | 10 cells | — | Wavelength = c_eff / f₀ |
| Courant number | ≈ 0.175 | — | Well below 1/√2 stability limit |

These values are identical to the FPGA `fdtd_engine.v` implementation so the
Python animation can be compared directly against FPGA output.

### Source injection — soft source

Sources use **soft injection** (`Ex[mask] += v`) rather than hard injection
(`Ex[mask] = v`). This matches the FPGA and means reflected waves can pass
back through the source cells without creating artificial re-reflections. The
waveform is a ramp-up sine to suppress startup transients:

```
v(n) = A × (1 − exp(−n × FREQ)) × sin(2π × FREQ × n × Δt + phase)
```

### Boundary conditions — PML

The outer 10-cell border is a **Perfectly Matched Layer** with a cubic
conductivity ramp. Outgoing waves are absorbed with < 1 % reflection.
Alternatively, `BOUNDARY_TYPE = "pec"` enforces metallic walls (E = 0).

---

## Physical scale — 50 cm × 50 cm conducting paper

The 256 × 256 grid maps onto the physical conducting paper (50 cm × 50 cm),
but the simulation's internal physics correspond to a far larger scale.
The derivation below starts from the normalised wave speed `C_SPEED = 0.5`
and the speed of light, treating **1 simulation time-step = 1 second**.

### Simulated cell size

In the simulation the wave travels at `C_SPEED = 0.5` cells per time-step.
Setting 1 step = 1 s and equating to the speed of light:

```
c_real  =  3 × 10⁸ m/s
c_sim   =  C_SPEED  =  0.5 cells/s

→  1 cell  =  c_real / c_sim  =  3×10⁸ / 0.5  =  6×10⁸ m  (600 000 km)
```

The physical paper cell spacing (50 cm ÷ 256 ≈ 1.95 mm) is the electrode
pitch on the desk-top apparatus; the simulation physics correspond to the
much larger scale above.

### Simulated domain, wavelength, and wave speed

| Quantity | Calculation | Result |
|---|---|---|
| Simulated cell size | c / C_SPEED = 3×10⁸ / 0.5 | **6×10⁸ m per cell** |
| Simulated paper area | 256 × 6×10⁸ m | **≈ 1.54×10¹¹ m  ≈ 1 AU** |
| Wavelength (10 cells) | 10 × 6×10⁸ m | **6×10⁹ m  (6 billion km)** |
| Wave frequency | FREQ = 0.05 cycles/step = 0.05 Hz | **0.05 Hz** |
| Cross-check f = c/λ | 3×10⁸ / 6×10⁹ | **= 0.05 Hz ✓** |
| Wave speed | c | **3×10⁸ m/s** |

The simulation therefore models EM wave propagation across a region
**approximately 1 AU (Earth–Sun distance) wide**, with a wavelength of
**6 billion km** and a frequency of **0.05 Hz** — using the 50 cm paper
as a scaled physical analogue.

### What c\_eff = 0.5 means

`c_eff = 0.5` is a **dimensionless FPGA numerical parameter**, not a physical
material constant. It sets the wave to travel at half the maximum CFL-stable
speed, which:

- Keeps the Q3.13 fixed-point update coefficients (CB = 0.35, CE = 0.0875)
  well within the representable range without overflow.
- Gives a Courant number of ≈ 0.175, comfortably below the 2-D stability
  limit of 1/√2 ≈ 0.707.
- Directly sets the simulated cell size: 1 cell = c / c_eff = 6×10⁸ m.

---

## Requirements

```
Python 3.8+
numpy
matplotlib
```

Install with:

```bash
pip install numpy matplotlib
```

---

## How to run

```bash
python fdtd_fullwave_simulation.py
```

The animated figure opens immediately. The title bar shows the current step,
physical time, frequency, wavelength, CFL number, and boundary type.

---

## Configuration

All user settings are in the `USER CONFIGURATION` block at the top of the file.
No other lines need to be changed.

### Sources — `SOURCES` list

Add up to ~15 sources as dicts in the `SOURCES` list:

```python
SOURCES = [
    {
        "type":   "line",   # "point" | "line" | "area"
        "x":      -15,      # centre x in grid units (0 = centre, range ±SPAN)
        "y":      0,        # centre y in grid units
        "len":    20,       # length along primary axis (line / area)
        "wid":    4,        # width along secondary axis (area only)
        "orient": "v",      # "h" horizontal | "v" vertical  (line only)
        "field":  "Ex",     # "Ex" | "Ey" — which component is driven
        "phase":  0.0,      # phase offset in radians relative to source 0
        "enable": True,
    },
    # copy block above and set enable: True to add more sources
]
```

Two sources with `"phase": np.pi` produces a visible interference pattern.

### Other options

| Parameter | Default | Effect |
|---|---|---|
| `IMAX` | 256 | Grid size (IMAX × IMAX cells) |
| `FREQ` | 0.05 | Normalised frequency — keep < 0.1 to avoid dispersion |
| `VF_AMP` | 1.0 | Peak source amplitude |
| `BOUNDARY_TYPE` | `"pml"` | `"pml"` absorbing / `"pec"` metallic walls |
| `PML_WIDTH` | 10 | PML thickness in cells (8–15 typical) |
| `PML_SIGMA_MAX` | 0.5 | PML absorption strength (0.3–0.8 recommended) |
| `GROUND_EXIST` | `False` | Enable a grounded (E = 0) conductor |
| `SHOW_EX/BZ/EMAG/POYNTING` | `True` | Toggle each display panel |
| `CLIM_E / CLIM_B` | 0.5 / 0.3 | Colour axis limits for E and B panels |
| `N_STEPS_PER_FRAME` | 4 | FDTD steps between animation frames |
| `N_FRAMES` | 300 | Total animation frames |

---

## Relation to the FPGA implementation

This script is the **software golden reference** for the Pynq-Z1 FPGA design
described in the FDTD Implementation Guide. It replicates the same algorithm:

- Same Yee leap-frog update order (Bz → Ex → Ey)
- Same numerical constants (CB, CE, f₀, Δt)
- Same soft source injection as `fdtd_engine.v` (Section 5.2)
- Same PML boundary approach (conductive medium decay)

To verify FPGA output, run this script and compare the field snapshots against
field arrays read back from the FPGA BRAMs over MMIO (see `read_field.py` in
the guide, Section 4.5).

---

## Commit history

| Hash | Description |
|---|---|
| `dc0bf95` | Match FPGA constants, add multi-source list, switch to soft injection |
| `b2cfa88` | Fix four bugs: dead PML code, roll wrap-around, unused mask, inconsistent Poynting formula |
| `5a00563` | Add files via upload |
