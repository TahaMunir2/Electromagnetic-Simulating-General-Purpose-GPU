# ===========================================================================
#  constraints.xdc  —  timing constraints for timing-analysis-only run
#  Target: xc7a100tcsg324-1  (Arty A7-100T)
#  Change -period to match your actual board clock.
# ===========================================================================

# 100 MHz system clock  →  period = 10 ns
# Try 150 MHz (6.667 ns) or 200 MHz (5 ns) once 100 MHz passes.
create_clock -period 10.000 -name sys_clk [get_ports clk]

# Relax all I/O paths — we only care about register-to-register timing
set_false_path -from [get_ports rst_n]
set_false_path -to   [all_outputs]
set_false_path -from [all_inputs]
