Second MVP:

Design constraints:
- 2D Ey, Ex and Bz
- Q3.13
- CORDIC Engine to generate sin waves for hard source (Xilinx provides an IP for this)
- 192x192 grid for Ey and Bz
- Boundaries zeroed causing reflection

Modules:
- BRAM - Yi - Update to support 3 fields + Poynting Vector and Initialised an Vivado IP
- Poynting Vector Calculator - Yi
- FDTD Solver - Taha - Update to support 2D

By Sunday:
- Individual Modules completed
- Update Top and Integrate
- Starting working on porting onto FPGA

Conclusion:
- We ended up achieving everything above + UPML
