# Step-by-step SMI-PI build (v4.1, space-time, gated)

One script per build stage. Run them in order. Each one runs a single stage of
`build_all.sh` and then prints its result. Only go on to the next script if
the check passes. Each script can be run from any folder.

| Script | Stage | Check |
|---|---|---|
| `01_vendor.sh`  | copy ADI HDL library | `STEP 1 vendor: PASS` |
| `02_libip.sh`   | ADI library IP | `STEP 2 libip: PASS` |
| `03_smi.sh`     | Vitis HLS SMI-PI filter (8 C-sim scenarios) | `SMI_PI_HLS: PASS`, clock <= 10 ns, II 1 |
| `04_ip.sh`      | gnss_passthrough IP | `IP_PACKAGE_OK` |
| `05_project.sh` | Vivado project | `CREATE_RESULT: PASS` |
| `06_build.sh`   | bitstream + XSA | `BUILD_TIMING: MET`, `BUILD_RESULT: PASS` |
| `07_sw.sh`      | application ELF (`--defines GNSS_DEPLOY_AUTO_TX` for the SD-card build that transmits at power-on) | `SW_RESULT: PASS` |
| `08_fsbl.sh`    | FSBL ELF | `FSBL_RESULT: PASS` |
| `09_boot.sh`    | BOOT.BIN | file listed |
| `10_report.sh`  | pass/fail report | verdict at the end |

If step 3 reports a clock above 10 ns: `SMI_PI_CLK_NS=8 ./03_smi.sh`

Step 6 times the AD9361 interface at 4 ns (61.44 MS/s). If it reports
`BUILD_TIMING: VIOLATED` with the failing paths on `rx_clk`, change the 4 to 5
in `Source/XDC/system.xdc` and run steps 5 and 6 again. 5 ns only covers
50 MS/s, so the default `gnss_band=1` (61.44 MS/s) is then outside what was
timed: type `gnss_band=2` (GPS, Galileo, GLONASS) or `gnss_band=3` (GPS,
Galileo, BeiDou) after every boot (see `BUILD_STEPS.md`, step 6).

If a step fails, run `tail -50 Build/Logs/<stage>.log`. After fixing it, run
the same script again.

Full walk-through from unzip to SD card, with every command: `BUILD_STEPS.md`
in the project folder. How to test on PuTTY and what to send back:
`TESTING_PUTTY.md`.
