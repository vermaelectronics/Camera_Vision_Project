# Waveform setup for tb_gnss_doa (loaded by sim_doa.tcl with DOA_SIM_GUI 1,
# or manually:  source waves_doa.tcl  in the xsim Tcl console)
set T /tb_gnss_doa
set D $T/dut/u_doa
add_wave_divider "AD9361 RX (l_clk, 12-bit right-aligned)"
add_wave $T/l_clk $T/adc_valid
add_wave -radix dec $T/adc_i0 $T/adc_q0 $T/adc_i1 $T/adc_q1
add_wave_divider "AXI-Lite (sys_cpu_clk)"
add_wave $T/aclk $T/awvalid $T/awready -radix hex $T/awaddr $T/wdata
add_wave $T/arvalid $T/rvalid -radix hex $T/araddr $T/rdata
add_wave_divider "DOA control"
add_wave -radix hex $D/r_ctrl $D/r_cal $D/r_dphase $D/r_decim
add_wave $D/proc_rst
add_wave_divider "CDC FIFO -> estimator input (Q1.15)"
add_wave $D/f_valid $D/r2_valid
add_wave -radix dec $D/r2_x0i $D/r2_x0q $D/r2_x1i $D/r2_x1q
add_wave -radix unsigned $D/in_count
add_wave_divider "Test generator"
add_wave $D/tg_valid
add_wave -radix dec $D/tg_x0i $D/tg_x0q $D/tg_x1i $D/tg_x1q
add_wave_divider "I-UESPRIT"
add_wave $D/i_res_valid
add_wave -radix dec $D/i_deg $D/i_phi $D/i_psi
add_wave -radix bin $D/i_flags
add_wave_divider "M-ESPRIT"
add_wave $D/m_res_valid
add_wave -radix dec $D/m_deg $D/m_phi $D/m_psi
add_wave -radix bin $D/m_flags
add_wave_divider "Averaged results"
add_wave -radix dec $D/i_avg_deg $D/m_avg_deg
add_wave -radix unsigned $D/i_avg_seq $D/m_avg_seq
run all
