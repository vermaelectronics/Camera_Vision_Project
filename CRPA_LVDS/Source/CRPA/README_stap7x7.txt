stap7x7: STAP-NLMS with 7 + 7 taps, reference delay 3.
Made from crpa27 (Source/CRPA/crpa27, exactly as delivered) by changing only
  fpga/stap_pi.h       ST_NT 8 -> 7, ST_REF 4 -> 3
  arm/gnss_aj_math.h   AJ_NT 8 -> 7, AJ_REF 4 -> 3
stap_pi.cpp and gnss_aj_math.c are identical to crpa27 (crpa27..31 are the same
code with different NT / REF, see README.txt). Image taps at delays 2, 3, 4.
