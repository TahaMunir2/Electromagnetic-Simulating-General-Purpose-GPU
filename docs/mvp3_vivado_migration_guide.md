# MVP3 Vivado Migration Guide

This document summarises every change made to the HDL since the pre-PML MVP2 baseline
and lists exactly what needs to be done to update the Vivado project.

---

## What the pre-PML version looked like

- `fdtd_solver` had `C_E` and `C_B` as input ports — constant damping coefficients
  wired in from the Block Design as fixed values
- `fdtd_engine` had a single `bz_left` port shared by both Ey and Ex updates
- The solver ran **3 passes** per iteration: Ey pass, Ex pass, Bz pass (3 × GRID_SIZE cycles)
- `Ey.sv`, `Ex.sv`, `Bz.sv` each had **2 pipeline stages** (difference reg + output reg)
- BRAM interface unchanged from MVP1

---

## Summary of all changes

### 1. New file: `pml.sv`

A combinational ROM that replaces the fixed `C_E`/`C_B` constants with
position-dependent UPML coefficients. Given a depth `d` (0–5, where 0 = interior,
5 = outermost PML cell), it outputs `ca`, `cb_e`, and `cb_bz` in Q3.13 fixed-point.

**Vivado action:** Add `pml.sv` to the project sources.

---

### 2. `fdtd_solver.sv` — three changes

#### a) `C_E` and `C_B` ports removed

The solver no longer takes `C_E` or `C_B` as inputs. Instead it instantiates three
`pml` modules internally (one each for Ey, Ex, Bz) and computes the correct
coefficients from the write-address position each cycle.

Old port list included:
```
input wire [DATA_WIDTH-1:0] C_E,
input wire [DATA_WIDTH-1:0] C_B
```
These are gone. Do not connect them in the BD.

#### b) `bz_left` split into `bz_left_ey` and `bz_left_ex`

Ey needs the Bz from the **row above** (fed via the `bz_adj` BRAM port).
Ex needs the Bz from the **column to the left** (fed from a `prev_bz` register).
These are now separate inputs to `fdtd_engine` so both can be computed in the same pass.

No new external BRAM ports — this is internal wiring only.

#### c) 3-pass → 2-pass (Ey and Ex parallelised)

Ey and Ex have no data dependency on each other (both only read Bz^n), so they now
update in the **same pass**. The solver completes one FDTD iteration in:

```
2 × GRID_SIZE cycles   (was 3 × GRID_SIZE)
```

For a 64×64 grid: **8 192 cycles** per iteration (was 12 288).

The `write_valid` offset also changed from `cell_addr >= 3` to `cell_addr >= 4`
due to the extra pipeline stage added in the engine (see below).

---

### 3. `fdtd_engine.sv` — `bz_left` port split

Matches the solver change. The single `bz_left` input is now:
```
bz_left_ey   — feeds the Ey submodule
bz_left_ex   — feeds the Ex submodule
```

**Vivado action:** If `fdtd_engine` is instantiated directly anywhere in the BD,
update the port connections. In normal use it is inside `fdtd_solver` so no BD
action needed.

---

### 4. `Ey.sv`, `Ex.sv`, `Bz.sv` — extra pipeline stage (timing fix)

Post-route timing showed WNS = −5.97 ns at 100 MHz. The critical path ran from
`counter` through phase arithmetic, through the Q3.13 multiply, and into the BRAM
enable — 20 logic levels, ~15–17 ns total.

Each engine module now has **3 pipeline stages** instead of 2:

| Stage | Old | New |
|-------|-----|-----|
| 1 | Register inputs (difference, field_old) | Same |
| 2 | Register output (field_new = ca×old + cb×diff) | Register truncated products (ca_reg, cb_reg) |
| 3 | — | Register output (field_new = ca_reg ± cb_reg) |

This splits the multiply+add combinational window into two shorter windows,
targeting ~8 ns each rather than ~16 ns combined.

The `write_valid` offset in `fdtd_solver` is bumped from 3 to 4 to account for
the extra latency cycle.

**Vivado action:** Replace `Ey.sv`, `Ex.sv`, `Bz.sv` in project sources.

---

### 5. `top_fdtd_system.sv` and `top_fdtd_hardware_wrapper.sv`

`C_E` and `C_B` removed from both module ports and internal wiring. Otherwise
structurally identical.

---

### 6. `fdtd_solver_bd_adapter.v`

`C_E_Q313` and `C_B_Q313` localparam declarations and their connections to
`fdtd_solver` removed. The adapter otherwise passes through the same BRAM and
control signals as before.

---

## Vivado Block Design checklist

| Task | Detail |
|------|--------|
| Add source | `pml.sv` |
| Replace sources | `fdtd_solver.sv`, `fdtd_engine.sv`, `Ey.sv`, `Ex.sv`, `Bz.sv` |
| Replace sources | `top_fdtd_system.sv`, `top_fdtd_hardware_wrapper.sv` |
| Replace adapter | `fdtd_solver_bd_adapter.v` |
| BD wiring — remove | Disconnect `C_E` and `C_B` constant blocks from the solver IP |
| BD wiring — remove | Delete the `C_E`/`C_B` Constant IP blocks if no longer used elsewhere |
| Re-validate BD | Run *Validate Design* — should have no critical errors |
| Re-synthesise | Timing should improve; WNS was −5.97 ns, expect < −1 ns or positive |

---

## What has NOT changed

- BRAM port interface (same address widths, same read/write signals)
- `source_in`, `source_valid`, `source_addr` interface
- `solver_enable` / `solver_done` handshake
- `bz_adj` and `ey_adj` ports
- `pml.sv` is self-contained — no new external connections required
- All testbenches pass (7/7 on `tb_fdtd_solver`, 8/8 on `tb_pml`)
