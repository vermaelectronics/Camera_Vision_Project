# app_ant_mvdr2: your MVDR2 firmware on the LVDS ant board

Use with the **Passthrough_MVDR** bitstream (gnss_passthrough.v at 0x43C10000).

## What it is
- **Unchanged MVDR2 modules** (from src_mvdr2_4): gnss_autostart, gnss_smi, gnss_passthrough,
  gnss_report, gnss_dash, gnss_info, gnss_mvdr_auto, gnss_mvdr_steer, gnss_l1, gnss_capture,
  gnss_txdma, gnss_config.h, command.c. So the RX gain auto-level, fast/slow gain and
  clip guard, AOLC -24 dBFS, TX1 34000 mdB, SMI-STAP (16 + 2 image taps, KEXP 8, loading 3,
  covariance averaging beta 1/4), events and reports are the ones that held the satellites.
- **Board layer** (new): `ant_platform.c` (VCXO, EMIO pins, band switches, LVDS init at
  61.44 MSPS), `main.c` (the proven SDT boot of the LVDS app + the MVDR2 main loop),
  the ad9361 / no-OS / xilinx_* drivers of the LVDS app (SPI InputClockHz fix).
- **Operator layer** (new): `gnss_ant.c` = gnss_tx / pass / stap, the main menu, aliases.

## Changes inside MVDR2 files (all small)
| File | Change |
|---|---|
| gnss_passthrough.h | base address fixed to 0x43C10000 |
| command.c / .h | old gnss_tx= body renamed gnss_tx_raw (used by pass=1); old gnss_tx? = gnss_tx_hw_print; `?` shows the new menu; new table entries; cmd_no unsigned |
| gnss_autostart.c | tx_silence() calls gnss_tx_raw(0) (same function as before) |
| gnss_l1.c | band-polarity probe at boot off (GNSS_BOOT_BAND_PROBE 0) |
| gnss_config.h | GNSS_AUTO_MVDR 0 (boot silent; stap=1 runs the automatic start) |
| gnss_info.c | address text 0x43C10000 |
| console.c | ',' or ' ' between parameters, at most 8, bounded buffer |

## Console
Boot: self-test, radio plan of preset 3 (LO 1582.5 MHz, 56 MHz, 61.44 MSPS:
GPS, Galileo, GLONASS, BeiDou), **gnss_tx=0 pass=0 stap=0**, main menu.

| Command | Effect |
|---|---|
| gnss_tx=1 | TX1 on at the operating attenuation, muted |
| pass=1 | raw RX1 on TX1; sets gnss_tx=1, stap=0 |
| stap=1 | MVDR2 automatic start (gain level, AOLC, SMI-STAP), TX1 on; sets gnss_tx=1, pass=1 |
| stap=0 | back to raw RX1 (pass=1) |
| pass=0 | mute, stap=0, TX1 stays on |
| gnss_tx=0 | everything off, TX1 silent |
| Enter | events + live status, then the main menu |
| rx?, noise?, noise=G, status?, stap?, guide? | aliases of gnss_rxgain?, gnss_floor?, gnss_spec=G, gnss_status?, (null+smi+supp), the MVDR2 menu |

All MVDR2 commands still work (gnss_check?, gnss_report?, gnss_band=, gnss_agc=, ...).
The three states are read from the hardware, so they stay right after those commands.
TX2 is held at 89 dB as in MVDR2 (unused).

## Vitis
1. Platform component from the new Passthrough_MVDR XSA (system_top.xsa).
2. New application component on it; delete the template sources; add every .c/.h of
   this folder (do not add the old antsdr_ant.c / gnss_mvdr.c).
3. Linker: libm (-lm, "m" in the libraries list) as in your MVDR2 project.
4. lscript.ld: use the same stack and heap as your MVDR2 project (src_mvdr2_4/lscript.ld);
   at least 0x10000 each. The MVDR2 reports nest ~1-1.5 kB frames; the template's 8 kB
   stack is too small.
