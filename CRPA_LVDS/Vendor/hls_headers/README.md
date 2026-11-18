# hls_headers: for the plain-g++ C simulation only

- `ap_*.h`, `etc/ap_private.h`: Xilinx arbitrary-precision types, from
  https://github.com/Xilinx/HLS_arbitrary_Precision_Types (Apache License 2.0;
  the licence text is at the top of every file).
- `hls_stream.h`: a minimal stand-in written for this project. It is enough
  for the testbench and nothing more.

`Automation/Linux/csim_gcc.sh` uses this folder when no Vitis HLS is installed.
The real build (`build_all.sh`, stage `hls`) always uses Vitis HLS 2023.2's
own headers, not these.
