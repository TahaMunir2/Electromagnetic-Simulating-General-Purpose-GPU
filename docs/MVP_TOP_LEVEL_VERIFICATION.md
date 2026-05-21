# MVP Top-Level Integration Verification

Date: 2026-05-21

This note summarizes the current MVP integration status for the 1D FDTD proof.

## What Changed

- Added a SystemVerilog top-level integration module: `src/hdl/top_fdtd_system.sv`.
- Integrated the existing project blocks:
  - `fsm_controller.sv`
  - `cordic_generator.v`
  - `bram_module.v`
  - `fdtd_engine.sv`
  - `Ey.sv`
  - `Bz.sv`
- Updated the top-level solver sequence to match the Python FDTD reference:
  - initialize BRAM fields to zero
  - request one CORDIC source sample per iteration
  - update all active Ey cells first
  - apply the hard source into Ey
  - update all active Bz cells using the updated Ey field
  - keep boundary cells at zero
- Fixed the Bz datapath sign/rounding form so its fixed-point result matches the reference equation:
  - `Bz[k] = Bz[k] + C_B * (Ey[k] - Ey[k+1])`
- Added top-level testbenches:
  - `tests/tb_top_fdtd_system.sv` for a quick integrated smoke test
  - `tests/tb_top_fdtd_reference.sv` for full-grid dump testing
- Added `scripts/compare_top_to_reference.py`, which runs the HDL top-level test with Icarus Verilog and compares all 64 Ey cells and all 64 Bz cells against a Python reference model.
- Updated `src/python/fdtd_reference.py` to avoid NumPy integer overflow and to clamp Q3.13 fixed-point values consistently.

## Verified Locally

The following checks were run in WSL:

```sh
iverilog -g2012 -Wall -o build/tb_bram_module.vvp tests/tb_bram_module.v src/hdl/bram_module.v
vvp build/tb_bram_module.vvp
```

Result:

```text
BRAM_PASS
```

```sh
iverilog -g2012 -Wall -o build/tb_cordic_generator.vvp tests/tb_cordic_generator.v src/hdl/cordic_generator.v
vvp build/tb_cordic_generator.vvp
```

Result:

```text
CORDIC_PASS
```

```sh
iverilog -g2012 -Wall -o build/tb_top_fdtd_system.vvp \
  tests/tb_top_fdtd_system.sv \
  src/hdl/top_fdtd_system.sv src/hdl/fsm_controller.sv \
  src/hdl/fdtd_engine.sv src/hdl/Ey.sv src/hdl/Bz.sv \
  src/hdl/bram_module.v src/hdl/cordic_generator.v
vvp build/tb_top_fdtd_system.vvp
```

Result:

```text
TOP_PASS
```

```sh
python3 scripts/compare_top_to_reference.py
```

Result:

```text
TOP_REFERENCE_PASS
Compared 64 Ey cells and 64 Bz cells after 4 iterations.
```

Additional tool checks:

```sh
python3 src/python/fdtd_reference.py
verilator --lint-only --sv --top-module top_fdtd_system \
  src/hdl/top_fdtd_system.sv src/hdl/fsm_controller.sv \
  src/hdl/fdtd_engine.sv src/hdl/Ey.sv src/hdl/Bz.sv \
  src/hdl/bram_module.v src/hdl/cordic_generator.v
yosys -q -p 'read_verilog -sv src/hdl/top_fdtd_system.sv src/hdl/fsm_controller.sv src/hdl/fdtd_engine.sv src/hdl/Ey.sv src/hdl/Bz.sv src/hdl/bram_module.v src/hdl/cordic_generator.v; hierarchy -top top_fdtd_system; proc; opt; stat'
```

Result:

- Python reference completed.
- Verilator lint passed.
- Yosys synthesis smoke check passed.

## Verified In Vivado

The top-level reference testbench was added to the Vivado project and run with XSim:

```tcl
set_property top tb_top_fdtd_reference [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {100us} -objects [get_filesets sim_1]
reset_simulation
launch_simulation
```

Result:

```text
TOP_REFERENCE_DUMP ey=top_fdtd_reference_ey.txt bz=top_fdtd_reference_bz.txt
TOP_REFERENCE_TB_DONE
```

The Vivado dump files were read from:

```text
E:/Vivado/Projects/desperate_yi/EM_Accelerator/EM_Accelerator.sim/sim_1/behav/xsim/top_fdtd_reference_ey.txt
E:/Vivado/Projects/desperate_yi/EM_Accelerator/EM_Accelerator.sim/sim_1/behav/xsim/top_fdtd_reference_bz.txt
```

The first active values from the Vivado dump were:

```text
Ey: 0 0 -1 1 26 -120 -351 941 -1 0 ...
Bz: 0 0 -1 -12 64 318 -1225 849 -2 0 ...
```

These match the Vivado CORDIC-calibrated reference source sequence:

```text
[8192, 0, -8193, -1]
```

Comparison result:

```text
Ey mismatches: 0
Bz mismatches: 0
```

## Current Status

The MVP top-level integration is functionally verified for the current 64-cell, Q3.13, 4-iteration proof path. The Vivado CORDIC IP introduces expected one-LSB quadrant rounding differences compared with an ideal sine source, and the FDTD output matches when those real IP samples are used as the reference input.
