First MVP:

Design constraints:
- 1D Ey and Bz
- Q3.13
- CORDIC Engine to generate sin waves for hard source (Xilinx provides an IP for this)
- 64 cell arrays for Ey and Bz
- Boundaries zeroed causing reflection

Modules:
- BRAM - Yi
- CORDIC Input Generator - Yi
- FDTD Solver - Taha

Python Reference File:
- Make a file that demonstrates what happens in a 64 x 64 yee grid with just Ey and Hz values with Q3.13 precision with a hard source input. 

By Friday:
- Individual Modules completed
- Create an FSM and Top Level Module to tie everything together
- Starting working on porting onto FPGA
