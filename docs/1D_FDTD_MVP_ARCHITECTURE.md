# 1D FDTD MVP Architecture Guide

This document explains the current 1D FDTD FPGA MVP for teammates who may not
have an electromagnetic simulation or FPGA background. It focuses on the active
root-level implementation used by the Vivado project, not the older placeholder
files in the nested MVP folder.

## Short Version

The project implements a small 1D electromagnetic wave simulator in FPGA logic.
It stores electric and magnetic field values in memory, generates a sine-wave
source, updates the fields cell by cell, and exposes a small hardware wrapper
that Vivado can synthesize, place, and route.

Current proof configuration:

- Grid: 64 cells
- Fields: `Ey` electric field and `Bz` magnetic field
- Number format: signed 16-bit Q3.13 fixed-point
- Source: sine wave from CORDIC
- Source location: cell 8
- Probe location: cell 8
- Boundary cells: held at zero
- Vivado target used in the latest run: `xc7z020clg400-1`
- Clock in the latest routed run: 100 MHz

## What FDTD Means Here

FDTD stands for finite-difference time-domain. In plain terms, it means:

1. Split space into small cells.
2. Store field values at each cell.
3. Advance time in steps.
4. At every time step, update each field using nearby field values.

For this MVP, the space is only one-dimensional. Instead of a 2D or 3D grid, the
design has a line of 64 cells:

```text
------+------+------+------+-----+------+
|  0   |  1   |  2   |  3   | ... |  63  |
+------+------+------+------+-----+------+
```

Each cell has two field values:

```text
Ey[k] = electric field value at cell k
Bz[k] = magnetic field value at cell k
```

The design updates these values over time. A sine-wave source is injected at one
cell, and that disturbance propagates through the 1D grid.

## The Update Equations

The hardware implements two simple update equations.

Electric field update:

```text
Ey[k] = Ey[k] + C_E * (Bz[k+1] - Bz[k])
```

Magnetic field update:

```text
Bz[k] = Bz[k] + C_B * (Ey[k] - Ey[k+1])
```

`C_E` and `C_B` are fixed coefficients. In the current hardware wrapper:

```text
C_E = 717
C_B = 2867
```

These are stored as Q3.13 fixed-point numbers.

## Fixed-Point Format: Q3.13

The FPGA design uses signed 16-bit fixed-point values instead of floating-point
values.

Q3.13 means:

- 1 sign bit
- 3 integer-range bits including the sign convention used by the signed value
- 13 fractional bits
- Total width: 16 bits

The important practical idea is that a real number is stored as an integer
scaled by `2^13`.

Examples:

```text
1.0   -> 8192
0.5   -> 4096
-1.0  -> -8192
```

When two Q3.13 values are multiplied, the raw product has too many fractional
bits, so the hardware shifts right by 13 bits:

```text
Q3.13 * Q3.13 -> wider product -> shift right 13 -> Q3.13
```

That shift happens in the `Ey` and `Bz` arithmetic modules.

## Active Source Tree

The current active root-level HDL implementation is:

```text
src/hdl/top_fdtd_hardware_wrapper.sv
src/hdl/top_fdtd_system.sv
src/hdl/fsm_controller.sv
src/hdl/cordic_generator.v
src/hdl/bram_module.v
src/hdl/fdtd_engine.sv
src/hdl/Ey.sv
src/hdl/Bz.sv
```

The current top-level tests are:

```text
tests/tb_top_fdtd_hardware_wrapper.sv
tests/tb_top_fdtd_system.sv
tests/tb_top_fdtd_reference.sv
scripts/compare_top_to_reference.py
```

There are also older placeholder files:

```text
src/hdl/fdtd_solver.v
src/hdl/top_fdtd_solver.v
```

Those files describe an earlier planned shape of the system. They are not the
working Vivado top-level path used in the latest routed build. The active
integrated design is `top_fdtd_system`, wrapped by `top_fdtd_hardware_wrapper`.

## Module Map

The active hardware hierarchy is:

```text
top_fdtd_hardware_wrapper
  |
  +-- top_fdtd_system
        |
        +-- fsm_controller
        |
        +-- cordic_generator
        |     |
        |     +-- cordic_0 Vivado IP when built with Vivado CORDIC enabled
        |
        +-- bram_module
        |
        +-- fdtd_engine
              |
              +-- ey
              |
              +-- bz
```

## Top Hardware Wrapper

File:

```text
src/hdl/top_fdtd_hardware_wrapper.sv
```

This module is the simple FPGA-facing wrapper used for synthesis and
implementation. It reduces the configurable simulation system to a fixed proof
configuration that can be built as hardware.

Inputs:

```text
clk
rst
start
```

Outputs:

```text
busy
done
iteration_count[15:0]
ey_probe[15:0]
bz_probe[15:0]
```

The wrapper hardcodes the current MVP parameters:

```text
CELLS = 64
DATA_WIDTH = 16
ADDR_WIDTH = 6
NUM_ITERATIONS = 4
PHASE_STEP = 16'h4000
SOURCE_ADDR = 6'd8
PROBE_ADDR = 6'd8
C_E = 16'sd717
C_B = 16'sd2867
```

Why this wrapper exists:

- It gives Vivado a clean top-level module.
- It avoids exposing every simulation parameter as a physical FPGA pin.
- It provides a small set of debug outputs for the MVP.

## Integrated System

File:

```text
src/hdl/top_fdtd_system.sv
```

This is the real system integration module. It connects:

- the top-level lifecycle state machine
- the iteration FSM
- the CORDIC source generator
- the field memory
- the FDTD arithmetic engine

Its main states are:

```text
TOP_IDLE
TOP_INIT
TOP_RUN
TOP_DONE
```

Meaning:

- `TOP_IDLE`: waiting for `start`
- `TOP_INIT`: clearing the field memories to zero
- `TOP_RUN`: generating source samples and updating fields
- `TOP_DONE`: simulation has completed

The module also exposes useful debug values:

```text
state_debug
iteration_count
cell_debug
source_sample
source_sample_valid
ey_probe
bz_probe
```

## Iteration Controller

File:

```text
src/hdl/fsm_controller.sv
```

This module decides when to generate the source and when to run the solver. It
does not directly calculate fields. It is a traffic controller.

Its states are:

```text
IDLE
SOURCE_GEN
SOLVE
DONE
```

Flow:

1. In `IDLE`, wait for a start pulse from `top_fdtd_system`.
2. In `SOURCE_GEN`, assert `cordic_enable`.
3. Wait for `cordic_done`.
4. In `SOLVE`, assert `solver_enable`.
5. Wait for `solver_done`.
6. In `DONE`, either start another iteration or finish the full run.

This separation keeps the source generator and solver from running at the wrong
time.

## Source Generator

File:

```text
src/hdl/cordic_generator.v
```

The source generator produces the sine-wave sample that gets injected into the
grid. In Vivado, it can use the Xilinx CORDIC IP instance named `cordic_0`.

Interface:

```text
phase_in
phase_valid
sin_out
cos_out
out_valid
```

The top system keeps a phase accumulator. Each time the controller asks for a
new source sample, the top system adds `phase_step` to the phase accumulator and
sends that phase into the CORDIC.

Current wrapper setting:

```text
PHASE_STEP = 16'h4000
```

That means the test source walks through quarter-cycle phase points.

The CORDIC output used by the FDTD solver is `sin_out`. `cos_out` is available
but not used by the current FDTD update path.

## Field Memory

File:

```text
src/hdl/bram_module.v
```

This module stores the `Ey` and `Bz` field arrays.

It contains four memory arrays:

```text
ey_mem_0
ey_mem_1
bz_mem_0
bz_mem_1
```

There are two copies of `Ey` and two copies of `Bz` so the solver can read two
neighboring values in the same cycle.

For example, to update `Ey[k]`, the engine needs:

```text
Bz[k]
Bz[k+1]
```

To update `Bz[k]`, the engine needs:

```text
Ey[k]
Ey[k+1]
```

The memory module gives the system two read ports per field by replicating the
arrays. Whenever the design writes an `Ey` value, it writes both `ey_mem_0` and
`ey_mem_1`. Whenever it writes a `Bz` value, it writes both `bz_mem_0` and
`bz_mem_1`.

Important Vivado note:

Although this module is named `bram_module`, the latest Vivado run implemented
these tiny memories as distributed RAM, also called LUTRAM, not physical block
RAM tiles.

Why:

```text
4 memories * 64 cells * 16 bits = 4096 bits total
```

That is small enough that Vivado maps it into LUTs. This is why the latest
report shows:

```text
Block RAM Tile: 0
LUT as Distributed RAM: 88
```

So the design has a memory module, but it does not currently use physical BRAM
resources.

## Arithmetic Engine

Files:

```text
src/hdl/fdtd_engine.sv
src/hdl/Ey.sv
src/hdl/Bz.sv
```

`fdtd_engine` is a small wrapper around the two arithmetic modules:

```text
ey
bz
```

The `ey` module computes:

```text
Ey[k] = Ey[k] + C_E * (Bz[k+1] - Bz[k])
```

The `bz` module computes:

```text
Bz[k] = Bz[k] + C_B * (Ey[k] - Ey[k+1])
```

Both modules are clocked. They register intermediate values and produce updated
field values after pipeline latency. That is why the top system has solver
states named:

```text
SOLVER_READ
SOLVER_PIPE1
SOLVER_PIPE2
SOLVER_WRITE
```

Those states give the memory and arithmetic path enough cycles to line up before
writing the new value back to memory.

## Solver Passes

One complete FDTD iteration has two field passes:

```text
FIELD_EY
FIELD_BZ
```

During `FIELD_EY`, the system walks across active cells and writes updated `Ey`
values.

During `FIELD_BZ`, the system walks across active cells and writes updated `Bz`
values.

The active cell range is:

```text
1 through 62
```

The boundary cells are:

```text
0 and 63
```

Those are left at zero for this MVP.

## One Iteration Step By Step

This is the practical story of one simulation iteration.

1. The wrapper receives `start`.
2. `top_fdtd_system` leaves `TOP_IDLE`.
3. `TOP_INIT` writes zero into every `Ey` and `Bz` memory location.
4. `top_fdtd_system` starts the iteration FSM.
5. `fsm_controller` enters `SOURCE_GEN`.
6. `top_fdtd_system` sends a phase value to `cordic_generator`.
7. `cordic_generator` returns a sine sample.
8. `top_fdtd_system` stores that sample as `source_sample`.
9. `fsm_controller` enters `SOLVE`.
10. The solver starts at cell 1.
11. In the `FIELD_EY` pass, the engine reads `Bz[k]` and `Bz[k+1]`.
12. The `ey` arithmetic module calculates the new `Ey[k]`.
13. If the current cell is the source cell, the design writes the source sample
    instead of the calculated value. This is the hard source injection.
14. After cell 62, the solver switches to `FIELD_BZ`.
15. In the `FIELD_BZ` pass, the engine reads the updated `Ey[k]` and
    `Ey[k+1]`.
16. The `bz` arithmetic module calculates the new `Bz[k]`.
17. After cell 62, the solver raises `solver_done`.
18. `iteration_count` increments.
19. The iteration FSM either requests another CORDIC source sample or finishes.
20. After the requested number of iterations, `done` becomes high.

## Why Ey Updates Before Bz

The Python reference and the hardware both use this sequence:

```text
update Ey using old Bz
apply hard source into Ey
update Bz using updated Ey
```

This ordering matters. If `Bz` used the old `Ey` instead of the updated `Ey`,
the output would not match the reference model.

## Probe Outputs

The hardware wrapper exposes:

```text
ey_probe
bz_probe
iteration_count
```

The probe address is currently fixed:

```text
PROBE_ADDR = 6'd8
```

That is the same as the source address in the current wrapper. This lets the
Vivado run expose a small amount of internal simulation state without exporting
all 64 cells.

## Verification Flow

The project has several test layers.

Unit tests:

```text
tests/tb_bram_module.v
tests/tb_cordic_generator.v
```

Integrated smoke test:

```text
tests/tb_top_fdtd_system.sv
```

Full-grid dump test:

```text
tests/tb_top_fdtd_reference.sv
```

Python comparison script:

```text
scripts/compare_top_to_reference.py
```

The comparison script:

1. Compiles the HDL top-level reference test with Icarus Verilog.
2. Runs the simulation.
3. Dumps all 64 final `Ey` values.
4. Dumps all 64 final `Bz` values.
5. Builds the same expected result in Python.
6. Compares every cell.

Expected pass message:

```text
TOP_REFERENCE_PASS
Compared 64 Ey cells and 64 Bz cells after 4 iterations.
```

Vivado/XSim was also used to run the hardware wrapper testbench:

```text
tests/tb_top_fdtd_hardware_wrapper.sv
```

Expected pass message:

```text
HW_WRAPPER_PASS
```

## Vivado Implementation Status

The latest Vivado run successfully completed:

```text
simulation
synthesis
placement
routing
timing analysis
```

Headline routed result from the latest run:

```text
Clock: 100 MHz
Timing met: yes
WNS: 3.516 ns
WHS: 0.106 ns
Slice LUTs: 1169
Slice Registers: 1153
DSPs: 2
Block RAM: 0
```

The most honest summary is:

```text
The 1D FDTD MVP successfully simulates, synthesizes, places, and routes at
100 MHz with timing met. Final board-level pin constraints are still pending.
```

## Current Limitations

This is an MVP, not a final board product.

Known limitations:

- The design is 1D only.
- The grid is only 64 cells.
- The wrapper currently runs only 4 iterations.
- Boundary cells are hard-zeroed, which causes reflections.
- Only one source location is used.
- Only one probe location is exposed.
- The top-level IO pins still need real board `LOC` and `IOSTANDARD`
  constraints before reliable bitstream signoff.
- The memory module is logically BRAM-style, but the current small arrays map to
  LUTRAM, so physical BRAM usage is 0.
- The CORDIC source in Vivado may differ by one least significant bit compared
  with an ideal math sine due to IP rounding.

## How To Explain It To Someone Quickly

Use this version:

```text
The 1D FDTD MVP is a 64-cell FPGA wave simulator. It stores Ey and Bz field
arrays, generates a sine source with CORDIC, updates the fields cell by cell
using fixed-point arithmetic, and has been routed in Vivado at 100 MHz with
timing met.
```

If they ask about zero BRAM:

```text
The RTL has a BRAM-style memory module, but the arrays are so small that Vivado
implemented them as distributed RAM in LUTs, so physical BRAM usage is 0.
```

## Where To Start Reading The Code

Recommended order:

1. `src/hdl/top_fdtd_hardware_wrapper.sv`
2. `src/hdl/top_fdtd_system.sv`
3. `src/hdl/fsm_controller.sv`
4. `src/hdl/bram_module.v`
5. `src/hdl/fdtd_engine.sv`
6. `src/hdl/Ey.sv`
7. `src/hdl/Bz.sv`
8. `src/hdl/cordic_generator.v`
9. `tests/tb_top_fdtd_hardware_wrapper.sv`
10. `scripts/compare_top_to_reference.py`

That path starts with the hardware-facing interface, then walks inward to the
control logic, memory, math, source generator, and verification.
