# SFAP-PI v6.3 build status (from Build/ logs, 2026-11-16)

Uploaded: README_SFAP_v6.3.md, Automation/, Docs/, patches/, and a curated
subset of Build/ (logs, BOOT.BIN, system_top.xsa, ELFs). `Source/` was NOT
uploaded; the v6.2.2 -> v6.3 diffs are in `patches/`. Generated folders
(Build/vitis_ws, Build/ip_repo, Build/csim_gcc, Automation/Linux/.Xil) left out.

| Stage | Result |
|---|---|
| HLS sfap_pi 5.0 | PASS, est. 9.629 ns @ 10 ns target, II = 1 |
| HDL / Vivado | PASS, timing met: WNS 0.129 ns, WHS 0.011 ns |
| Utilisation | LUT 25121 / 53200 (47.2 %), FF 31201 (29.3 %), BRAM 23 / 140 |
| SW (FSBL + app_sfap_lvds, release) | PASS |
| BOOT.BIN | PASS, sha256 ced133a607ac94bd6c8ae30dfebdec6876fe65e3780b31acd1c0254ffdfb332b |

So of README section 5 "Not done": HLS synthesis (II/timing), Vivado
implementation/timing and the ARM build are now done. Still open: running on
the board (bench checks in README section 4).
