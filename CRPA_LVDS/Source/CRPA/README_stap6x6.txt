stap6x6: STAP-NLMS with 6 + 6 taps, reference delay 3.
Made from crpa27 (Source/CRPA/crpa27, exactly as delivered) by changing only
  fpga/stap_pi.h       ST_NT 8 -> 6, ST_REF 4 -> 3
  arm/gnss_aj_math.h   AJ_NT 8 -> 6, AJ_REF 4 -> 3
stap_pi.cpp and gnss_aj_math.c are identical to crpa27 (crpa27..31 are the same
code with different NT / REF, see README.txt). Image taps at delays 2, 3, 4.
Taps 0..5: the reference RX1 tap 3 has 3 taps before it and 2 after it.
Same firmware features as the other STAP builds: Wiener start, NLMS, tracking,
Kalman weight tracker (on at boot, crpa_kalman=0 to switch off). Block design
with the lower-power option C (as stap7x7).
