# SFAP-PI v6.3 build status (from Build/ logs, 2026-11-16)

Contents: README_SFAP_v6.3.md, Source/ (HLS, HDL + testbench, block design,
IP packaging, XDC, firmware), test/ (C-sim and HDL testbench results),
Automation/, Docs/, patches/, and a curated subset of Build/ (logs, BOOT.BIN,
system_top.xsa, ELFs). Generated folders left out: Build/vitis_ws,
Build/ip_repo, Build/csim_gcc, Automation/Linux/.Xil, and
Source/HLS/sfap_pi/build/ (~100 MB of Vitis HLS output) except its
syn/report/csynth.rpt and sfap_pi_csynth.rpt.

| Stage | Result |
|---|---|
| C-sim (test/csim_results_v6.3.txt) | PASS, scenarios 1-23 |
| HDL testbench (test/hdl_tb_results_v6.3.txt) | HDL_TB: PASS |
| HLS sfap_pi 5.0 | PASS, est. 9.629 ns @ 10 ns target, II = 1 |
| HDL / Vivado | PASS, timing met: WNS 0.129 ns, WHS 0.011 ns |
| Utilisation | LUT 25121 / 53200 (47.2 %), FF 31201 (29.3 %), BRAM 23 / 140 |
| SW (FSBL + app_sfap_lvds, release) | PASS |
| BOOT.BIN | PASS, sha256 ced133a607ac94bd6c8ae30dfebdec6876fe65e3780b31acd1c0254ffdfb332b |

So of README section 5 "Not done": HLS synthesis (II/timing), Vivado
implementation/timing and the ARM build are now done. Still open: running on
the board (bench checks in README section 4).
