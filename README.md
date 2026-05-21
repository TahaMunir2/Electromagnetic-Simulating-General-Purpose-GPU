# Electromagnetic-Simulating-General-Purpose-GPU

GPU and FPGA-accelerated electromagnetic simulation framework, focusing on 1D FDTD (Finite-Difference Time-Domain) implementation.

## Project Layout

- `src/` - simulator implementation
  - `hdl/` - Hardware description language modules
  - `python/` - Python reference implementations and utilities
- `include/` - shared headers and public interfaces
- `tests/` - verification and regression tests
- `examples/` - runnable examples and sample setups
- `docs/` - design notes, setup guides, and equations
- `scripts/` - developer utilities and automation
- `data/` - small sample inputs and test fixtures

## MVP: 1D FDTD Implementation

Current focus on a 1D FDTD solver with the following specifications:

- **Field Components**: 1D Ey and Bz
- **Precision**: Q3.13 fixed-point
- **Cell Count**: 64-cell arrays for Ey and Bz
- **Wave Source**: CORDIC-based sine wave generator
- **Boundary Conditions**: Zero boundaries (causing reflections)

See [1D FDTD Reference](docs/1d_fdtd_reference.md) for detailed specifications.
For a teammate-friendly walkthrough of the active FPGA architecture, see
[1D FDTD MVP Architecture Guide](docs/1D_FDTD_MVP_ARCHITECTURE.md).

## Status

Scaffold and template structure in place. Core modules being developed:
- BRAM module (Yi)
- CORDIC Input Generator (Yi)
- FDTD Solver (Taha)
- Python reference implementation
- FSM and Top-level module integration

## Getting Started

Refer to individual module READMEs in the `src/` directory for setup and compilation instructions.
