# Design 1 PYNQ-Z1 HDMI Output

Use `design1_ray_unit_top.sv` for timing-only implementation runs.

Use `design1_ray_unit_hdmi_top.sv` for board output on the PYNQ-Z1 HDMI TX connector.

## Vivado Setup

1. Change the project part/board to PYNQ-Z1 / `xc7z020clg400-1`.
2. Add these HDL files to Design Sources:
   - `design1/D1_wrapper/design1_ray_unit_hdmi_top.sv`
   - `design1/D1_wrapper/design1_video_timing_640x480.sv`
   - `design1/D1_wrapper/design1_heightmap_bram.sv`
   - `design1/design1_*.sv`
3. Add `design1/D1_wrapper/pynq_z1_hdmi.xdc` to Constraints.
4. Disable or remove `wrapper/constraints.xdc` from this HDMI constraints set.
   That file is only for the timing-analysis wrapper and creates a conflicting
   100 MHz clock.
5. Set `design1_ray_unit_hdmi_top` as the top module.
6. Add Clocking Wizard IP named `clk_wiz_0`:
   - input clock: `125.000 MHz`
   - `clk_out1`: `25.000 MHz` or `25.175 MHz`
   - `clk_out2`: `125.000 MHz` or `125.875 MHz`
7. Add Digilent `rgb2dvi` IP named `rgb2dvi_0`, configured for 7-series.
8. Generate bitstream and program the board with a monitor connected to HDMI OUT.

`rst` is connected to BTN0 and is active high. Leave BTN0 unpressed for normal
operation; press it to reset the HDMI/render pipeline.

The Design 1 renderer latency is assumed to be 78 pixel-clock cycles. `design1_ray_unit_hdmi_top`
delays `hsync`, `vsync`, and data-enable by that amount so the video control
signals line up with the RGB output from `design1_ray_unit`.

## Mock Heightmap

`design1_heightmap_bram` initializes itself by default. With `USE_MOCK_DATA=1`,
each inferred BRAM powers up with a small synthetic terrain: a centered square
pyramid. This removes the Vivado `mem does not have driver` warning and
lets the HDMI renderer produce a visible image before a real map-loading path
exists.

To use an external hex file later, instantiate `design1_heightmap_bram` with:

```systemverilog
.USE_INIT_FILE (1'b1),
.INIT_FILE     ("heightmap_64x64.hex"),
.USE_MOCK_DATA (1'b0)
```

The hex file should contain one signed 16-bit value per line, in row-major
`{y, x}` address order. For the current `GRID_N=64`, that is 4096 lines.
