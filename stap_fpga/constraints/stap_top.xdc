# Clock constraint for stap_top.
# Default (Zynq-7020): FOLD=2, 120 MHz clock -> 60 MSPS (the MATLAB sample rate).
# FOLD=4 variant: 240 MHz (4.167 ns).  FOLD=1: 60 MHz (16.667 ns).
create_clock -name clk -period 8.333 [get_ports clk]
