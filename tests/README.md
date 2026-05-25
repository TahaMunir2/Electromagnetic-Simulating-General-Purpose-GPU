# Tests

Verification, regression, and numerical accuracy tests live here.

## Physical Wave Probe

`tb_fdtd_physical_wave_probe.sv` is a simulation-first check for the MVP2
2D FDTD pipeline. It injects an impulse source at the center of the grid,
runs multiple frames, samples virtual probe points along the wave path, and
checks that the render buffer is written in both `|E|` and `|S|` modes.

Run it from the repository root with:

```bash
mkdir -p build && iverilog -g2012 -I src/hdl -o build/tb_fdtd_physical_wave_probe.vvp \
  src/hdl/Ey.sv src/hdl/Ex.sv src/hdl/Bz.sv src/hdl/fdtd_engine.sv \
  src/hdl/fdtd_solver.sv vivado/fdtd_solver_bd_adapter.v \
  tests/tb_fdtd_physical_wave_probe.sv && \
vvp build/tb_fdtd_physical_wave_probe.vvp
```

The test passes when the near probe responds before the mid probe, the fields
stay below saturation, every `s_mag` cell is written, and `|E|`/`|S|` produce
different render-buffer checksums.
