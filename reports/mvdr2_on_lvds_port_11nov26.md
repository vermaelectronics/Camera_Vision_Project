# MVDR2 design ported to the LVDS ant board (11 Nov 2026)

Package: E310_MVDR2_LVDS.zip, which contains Passthrough_MVDR (HDL) and app_ant_mvdr2 (firmware).

## Why
- The first LVDS MVDR integration lost every satellite under a 56 MHz barrage at 0 dB. The old MVDR2 design held them, cold start included.
- The RTL is not the cause. mvdr_top, mvdr_regs, stap_smi, cov2x2_accum, iq_corr, stap_lms and mvdr_xdom are byte-identical between the two designs (cmp).
- The cause is the firmware. The first LVDS app had:
  - a fixed 58 dB RX gain, with no levelling, fast/slow gain or clip guard;
  - AOLC off, and TX1 at 10 dB instead of 34000 mdB;
  - no SMI covariance averaging (MOD-31 β 1/4);
  - no event monitor or reports.

## HDL (Passthrough_MVDR)
- `rtl/gnss_passthrough.v` is the MVDR2 top, copied unchanged. `gnss_passthrough_bd.v` only adds the clock/reset interface attributes.
- Wiring matches the MVDR2 block design: ADC enable/valid/data, DAC enable/valid, DMA data from tx_upack, dac_data to axi_ad9361.
- Base address **0x43C10000**, because 0x43C00000 is the ant VCXO controller.
- No TDD engine, ADC DMA sync start off, impl strategy Performance_ExplorePostRoutePhysOpt (from the timing fix).
- Simulation (iverilog `tb/tb_gnss_passthrough.v`), 0 errors:
  - IDs 0x47435031 / MVDR 0x0001000F / SMI INFO 0x03081053;
  - DMA path correct when off;
  - RX1→TX1 bit-exact (×16) on every dac_valid, with no discontinuity and no FIFO over/underflow.
- Old gnss_mvdr.v is retired.

## Firmware (app_ant_mvdr2)
- **MVDR2 modules, unchanged:** autostart, smi, passthrough, report, dash, info, mvdr_auto/steer, l1, capture, txdma, config, command.
- **Small edits:**
  - base address;
  - old gnss_tx= renamed gnss_tx_raw (used by pass=1);
  - `?` shows the new menu;
  - band probe off at boot;
  - GNSS_AUTO_MVDR 0;
  - cmd_no unsigned;
  - console: ',' or ' ' as separator, at most 8 parameters.
- **Board layer:** `ant_platform.c` (VCXO 23000 manual, EN_AGC, band switches 86..93 H=0 L=1, LVDS init at 61.44 MSPS) and `main.c`.
  - main.c uses the SDT boot of the LVDS app, followed by the MVDR2 self-test and polled main loop: SMI poll every pass, background tick every 10 ms.
  - The UART is accessed through its registers.
- **Operator layer (`gnss_ant.c`):**
  - boot: gnss_tx=0 pass=0 stap=0, radio plan of preset 3 (LO 1582.5 MHz, 56 MHz, 61.44 MSPS);
  - gnss_tx=1: TX1 on, muted;
  - pass=1: gnss_tx=1, stap=0, raw RX1;
  - stap=1: gnss_tx=1, pass=1, running gnss_autostart_run();
  - stap=0: back to raw RX1;
  - pass=0: mute;
  - gnss_tx=0: everything off.
  - The three states are read back from the hardware: STATUS pass_en/mvdr_sel, CONTROL mute, DAC enable, TX1 attenuation.
- **Aliases:** rx? = gnss_rxgain?, noise? = gnss_floor?, noise=G = gnss_spec=G, status? = gnss_status?, stap? = null + smi + supp, guide? = the MVDR2 menu.
- **Enter:** events and live status, then the main menu.
- TX2 is held at 89 dB, as in MVDR2.
- **Build notes:** link libm. Use the MVDR2 lscript.ld stack/heap (≥ 0x10000), because report frames are ~1–1.5 kB.
- **Host check:** gcc syntax and link with BSP stubs. The only unresolved symbols are BSP/driver ones (Xil_*, no_os_mdelay, xil_*_ops).

## Test to run
Run a 56 MHz barrage at 0 dB with `stap=1`, then read `gnss_report?`, and compare with the MVDR2 board, including a cold start.
