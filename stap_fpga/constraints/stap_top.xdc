# Clock constraint for stap_top. 240 MHz -> FOLD=4 at 60 MSPS (the MATLAB sample rate).
# For FOLD=1 use the sample clock (60 MHz -> 16.667 ns).
create_clock -name clk -period 4.167 [get_ports clk]
