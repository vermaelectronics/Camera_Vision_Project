# Step-by-step SMI-PI build

One script per build stage. Run them in order. Each one runs a single stage of
`build_all.sh` and then prints its result. Only go on to the next script if
the check passes. Each script can be run from any folder.

| Script | Stage | Check |
|---|---|---|
| `01_vendor.sh`  | copy ADI HDL library | `STEP 1 vendor: PASS` |
| `02_libip.sh`   | ADI library IP | end of `libip.log` |
| `03_smi.sh`     | Vitis HLS SMI-PI core | `SMI_PI_HLS: PASS`, clock <= 8 ns, II 2 |
| `04_ip.sh`      | gnss_passthrough IP | `IP_PACKAGE_OK` |
| `05_project.sh` | Vivado project | `CREATE_RESULT: PASS` |
| `06_build.sh`   | bitstream + XSA | `BUILD_TIMING: MET`, `BUILD_RESULT: PASS` |
| `07_sw.sh`      | application ELF | `SW_RESULT: PASS` |
| `08_fsbl.sh`    | FSBL ELF | `FSBL_RESULT: PASS` |
| `09_boot.sh`    | BOOT.BIN | file listed |
| `10_report.sh`  | pass/fail report | verdict at the end |

If step 3 reports a clock above 8 ns: `SMI_PI_CLK_NS=7 ./03_smi.sh`

If a step fails, run `tail -50 Build/Logs/<stage>.log`. After fixing it, run
the same script again.
