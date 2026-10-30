# Testing SMI-PI v4.1 on PuTTY, and what to send back

This guide sets PuTTY up so every line the board prints is saved to a file.
Then come the tests. Each one says what to type, what you should see, and
what to write down. At the end there is a **report template**: fill it in and
paste it into your next message, together with the `gnss_report?` blocks.
That gives me exactly what I need for the next improvement.

---

## 1. PuTTY settings (once)

Find the COM port: Windows **Device Manager → Ports (COM & LPT)**. The E310
console shows up as a "USB Serial Port (COMx)".

| PuTTY category | Setting |
|---|---|
| Session | Connection type **Serial**, Serial line **COMx**, Speed **115200** |
| Connection → Serial | Data bits **8**, Stop bits **1**, Parity **None**, Flow control **None** |
| Terminal | tick **Implicit CR in every LF** (the board ends lines with LF only) |
| Window | Columns **160**, Lines of scrollback **50000** |
| Session → Logging | **All session output**; Log file name `C:\AJlogs\e310_&Y&M&D_&T.log`; tick **Flush log file frequently**. (Create the folder `C:\AJlogs` first.) |
| Session | Saved Sessions: type `E310`, press **Save** |

Every time you connect, a new time-stamped log file is created, so you never
have to copy from the screen. To copy a block from the screen, select it with
the mouse (that copies it) and paste it with Ctrl+V in your message.

Type commands exactly as shown and press Enter. By default the board prints
only jammer events; `gnss_aj_log=30` adds a status line every 30 s, which
waits while you type, so it never runs through a command.
Jammer events also wait while you type; `gnss_aj_events=0` silences the
console completely (`gnss_report?` still lists every event). `?` shows the menu,
`help?` lists every command.

---

## 2. The lines to read

| Line / field | Meaning | Good value |
|---|---|---|
| `[AJ 12.345 s]` | time since the anti-jam engine started | – |
| `TX1 ON: the receiver's time to fix counts from here` | the moment the receiver starts getting a signal | start your stopwatch here |
| `*** JAMMER ON ***  J/N +30.2 dB, WIDEBAND ...` | jammer detected. J/N = how far it is above the board's noise floor | – |
| `detected because ...` | coherence (both antennas see it), power, or the RX gain cut | coherence, normally |
| `NULLED in 16 ms` | time from detection until the jammer was removed | < 100 ms |
| `suppression 45.0 dB` | how much the jammer was reduced (capped at 60) | ≥ J/N |
| `residual 0.3 dB` | output power above the plain noise; **0 = jammer completely removed** | < 1 dB (warning above 3) |
| `make-up +24 dB` | digital gain that restores the level after the RX gain cut | – |
| `RX 43/70 dB` | RX gain now / clean reference | – |
| `jammer OFF after 4.5 s ... filter bypassed, TX1 = RX1` | jammer gone, output is the plain RX1 signal again | – |
| `clear ... coherence -15.0 dB` | no jammer; coherence about −15 dB is pure noise | below −9 dB |
| `g2 +1.0 dB` | how much RX2 is boosted to match RX1 | within ±6 dB (more = check RX2 cable/antenna) |
| `checks/s`, `solves/s` | engine speed while clear / jammed | a few hundred checks/s; solves/s ≥ 50 (please report the value) |
| `gnss_band?` → `received?` | which GNSS systems the RF profile carries | all `OK` |

---

## 3. Tests

Use the **same receiver, antennas, cables and GNSS viewer** in every test.
Write down the jammer generator's setting in **dBm** (and frequency /
bandwidth) as well as the J/N the board prints. The two are different
things: J/N is relative to the board's noise.

### T0. Boot check (every new BOOT.BIN)

1. Power on with PuTTY connected. Look for:
   ```
   gnss_pt: present at 0x43c00000, version 4.5
   SELFTEST: PASS ...
   ANTI-JAM (SMI-PI 4.5.0): ON, bypassed until a jammer appears.
   ```
2. Type `gnss_band?`. All five signals must show `OK` and both FIRs `off`.
3. Type `gnss_report?` and keep the block.

### T1. Constellations, no jammer (fixes "no BeiDou / fewer satellites")

1. In the receiver, switch on GPS, Galileo, GLONASS **and BeiDou**. Check its
   manual: some track only 3 systems at once.
2. Type `gnss_tx=1`. Note the `[AJ ...] TX1 ON` time.
3. After 2 minutes write down, per system: number of satellites **seen** and
   **used**, and the C/N0 of the 3 strongest.
4. For comparison, connect the antenna directly to the receiver (no AntSDR)
   and write down the same numbers. Through the AntSDR they should be within
   ~1–3 dB C/N0 and the same satellites.
5. Optional: try `tx1_attenuation=65000` and `75000`. Keep the one with the
   highest C/N0 (then `tx1_attenuation=70000` to go back).

### T2. Time to fix (lock time)

For each receiver start type (hot, warm, cold, using the viewer's buttons),
three times:

1. `gnss_tx=0`, issue the start command in the viewer, then `gnss_tx=1`.
2. Write down the seconds from `TX1 ON` to **3D fix**.

Do the same once with the antenna directly on the receiver, to see the
receiver's own times. Hot start should be a few seconds. A cold start can
never be much faster than ~30 s (README_SMI_PI_v4.1.md §6).

### T3. Jammer level steps

Jammer: CW, then wideband (~55 MHz), centred near 1575 MHz. At each level
(10, 20, 30, 40, 50 dB J/N as printed, or your generator steps):

1. Switch the jammer on and wait 30 s.
2. Copy the `JAMMER ON`, `NULLED` and the next `JAMMED` status line.
3. Type `gnss_report?` and keep the block.
4. Write down satellites seen/used per system, average C/N0 and fix yes/no.
5. Switch the jammer off and check that `jammer OFF ... filter bypassed`
   appears.

### T4. Jammer on/off cycles

`gnss_aj_csv=500`, then jammer 30 s on / 30 s off, three times, at 30 dB and
at 50 dB. Afterwards `gnss_aj_csv=0`. Note whether the receiver kept its fix
through the switches. The `AJCSV,` lines in the log open in Excel (Data →
From Text/CSV, comma-separated; keep only the lines starting with `AJCSV`).

### T5. Jammer already on at power-on

Jammer on (30 dB), then power the board on. Look for `JAMMER ON` within the
first second after the banner and `g2 ... from the jammer` in
`gnss_report?`. Then `gnss_tx=1` and time to fix.

### T6. A/B comparisons (at the strongest jammer that still gives a fix)

| Type | Compares |
|---|---|
| `gnss_aj_mode=3` then `gnss_aj_mode=0` | without / with the image taps |
| `gnss_aj_mode=2` then `gnss_aj_mode=0` | old single weight / v4.1 |
| `gnss_smi=0` then `gnss_smi=1` | no anti-jam at all (expect satellites lost with a strong jammer) |

After each change, wait 20 s, then `gnss_report?` + satellites/C/N0.

### T7. CW on top of the GPS signal (the field test of 28-Oct, firmware 4.4.0)

Jammer: CW at 1575.4 MHz (the LO of `gnss_band=1` is 1582.5 MHz, so it is 7.1 MHz
below it; `gnss_band=0` puts it at the LO and the AD9361 then takes part of it away).
The same receiver, antennas and cables as before; TX attenuation 70 dB (`gnss_tx=1`;
write down if you use another value, 20 dB makes the output into the receiver 50 dB hotter).

1. No jammer first: `gnss_report?`, satellites seen/used, average C/N0.
2. Jammer on at the lowest level, then 5 dBm higher, up to the highest you use
   (for example 10, 15, 20, 25 dBm). At each: wait 30 s, then `gnss_report?`.
3. From each block copy: `jammer` (J/N), `suppression`, `residual`, `rx gain` (and the
   clean reference), `ADC peak`, `weights ... weight scale 2^n`. Then satellites seen/used
   per system, average C/N0 and whether you have a fix; for cold / hot start also the
   time to fix.
4. How to read it:
   * suppression about 60 dB or more and residual below +1 dB: the filter does its job;
     if satellites are still missing, they are the ones in the jammer's direction (a
     2-element array nulls one direction) or the receiver itself is overloaded;
   * J/N printed above about 72 dB, or `rx gain` at 1-5 dB with `ADC peak` near 2047:
     the jammer is too strong for the AD9361 itself: add an attenuator before the board;
   * `weight scale 2^0` or the `NOTE: the bitstream is older than 4.4` line at boot: the
     bitstream was not rebuilt: do the whole build (BUILD_STEPS.md), not only the firmware.
5. Switch the jammer off: `jammer OFF ... filter bypassed` must appear, and the satellites
   must come back within a few seconds.

### T8. Barrage (noise) jammer, 50 MHz wide, on 1575.4 MHz (firmware 4.5.0)

Set the radio the way the field test did: `gnss_band=5 1575.42 18 20 70 30` (LO 1575.42 MHz, 18 MS/s,
20 MHz, TX att 70 dB, RX gain 30 dB). Then `gnss_tx=1`. If you need a lower TX attenuation to get
the receiver to lock without a jammer (the field test used 34 dB), set it first and keep it.

1. No jammer: `gnss_report?`, satellites seen/used, average C/N0 (the reference).
2. Jammer on at "10", "20", "25", "30" (your generator's steps, 10 dB below its maximum of 40 for
   "30"). At each: wait 60 s, then `gnss_report?`.
3. Copy from each block: `jammer` (J/N), `suppression`, `residual`, `rx gain`, **`output level`**
   (new: should be between about -6 and +3 dB), `ADC peak`, `weights ... scale`, and the satellites
   seen/used per system and the average C/N0.
4. Expected from the simulation (README_SMI_PI_v4.5_barrage.md §4): J/N up to about 51 dB (jammer
   "20"): nothing lost; 56 dB: about 1 dB; 61 dB: about 3 dB; above 66 dB: 5 dB and more.
5. If `output level` is far below -6 dB, tell me the `rx gain` lines and the TX attenuation: the level hold or
   the make-up is not reaching the clean level.

---

## 4. Report template (copy, fill in, paste into your next message)

```
=== SMI-PI v4.1 TEST REPORT ===
Date:
Build: BOOT.BIN sha256 (Build/Logs/boot.log):           built with --defines:
Step 3 HLS: clock estimate ___ ns, II ___      Step 6: BUILD_TIMING MET / VIOLATED
Receiver model / firmware:                      GNSS viewer:
Constellations enabled in the receiver: GPS / Galileo / GLONASS / BeiDou (B1I? B1C?)
Antennas (type, active/passive, LNA, cable lengths RX1 / RX2):
Signal source (sky / simulator, level):
Jammer generator (model), type (CW / wideband), centre frequency, bandwidth:

T0  gnss_band? all OK?  yes / no (paste if no)

T1  no jammer, TX att ____ dB
            seen / used    strongest C/N0 (3)
    GPS      __ / __        __ __ __
    Galileo  __ / __        __ __ __
    GLONASS  __ / __        __ __ __
    BeiDou   __ / __        __ __ __
    (antenna directly on the receiver: GPS __/__ GAL __/__ GLO __/__ BDS __/__)

T2  time to fix from "TX1 ON" [s]     run1   run2   run3   (direct, no AntSDR)
    hot                               ____   ____   ____   ____
    warm                              ____   ____   ____   ____
    cold                              ____   ____   ____   ____

T3  generator  printed J/N  suppression  residual  NULLED in  sats used (G/E/R/C)  avg C/N0  fix
    CW  ___ dBm   ____        ____         ____      ____ ms    __/__/__/__          ____      y/n
    CW  ___ dBm   ____        ____         ____      ____ ms    __/__/__/__          ____      y/n
    WB  ___ dBm   ____        ____         ____      ____ ms    __/__/__/__          ____      y/n
    WB  ___ dBm   ____        ____         ____      ____ ms    __/__/__/__          ____      y/n

T7  CW 1575.4   dBm  J/N   suppression  residual  rx gain / ADC peak  weight scale  sats G/E/R/C  avg C/N0  fix
    ___            ____  ____         ____      ____ / ____         2^_           __/__/__/__   ____      y/n
    ___            ____  ____         ____      ____ / ____         2^_           __/__/__/__   ____      y/n

T8  barrage 1575.4  setting  J/N  suppression  residual  output level  rx gain / ADC peak  sats G/E/R/C  avg C/N0  fix
    ___            ____   ____   ____       ____          ____ / ____         __/__/__/__   ____      y/n
    ___            ____   ____   ____       ____          ____ / ____         __/__/__/__   ____      y/n

T4  kept fix through on/off?  30 dB: y/n   50 dB: y/n
T5  detected at boot? y/n, NULLED in ___ ms, time to fix ___ s
T6  mode 3 vs 0: residual ___ vs ___ dB; mode 2 vs 0: sats ___ vs ___; smi=0: sats ___

Anything strange (WARNING lines, "radio changed", DMA errors, resets):

Paste below: the gnss_report? block of the worst case, and the
JAMMER ON / NULLED / OFF lines.
=== END ===
```

The PuTTY log files in `C:\AJlogs\` contain everything. If a test goes
wrong, send the whole log of that session (or the part around the problem).

---

## The six RF profiles (`gnss_band=N`)

| N | LO | rate | RF bandwidth | carries |
|---|---|---|---|---|
| 0 | 1575.42 MHz | 30.72 MS/s | 18 MHz | GPS, Galileo, QZSS, SBAS, BeiDou B1C (L1 only) |
| **1 (default)** | 1582.5 MHz | 61.44 MS/s | 56 MHz | all: GPS, Galileo, BeiDou B1I + B1C, GLONASS |
| 2 | 1589.5 MHz | 50 MS/s | 40 MHz | GPS, Galileo, GLONASS (no BeiDou B1I) |
| 3 | 1568.25 MHz | 50 MS/s | 40 MHz | GPS, Galileo, BeiDou (no GLONASS) |
| 4 | 1601.72 MHz | 20.48 MS/s | 16 MHz | GLONASS G1 only |
| **5 = CUSTOM** | your value | your value | your value | whatever you choose (start-up: 1575.42 MHz, 30.72 MS/s, 18 MHz) |

`gnss_band?` lists them and shows which signals the active one carries.

**Profile 5 (CUSTOM) is yours.** Fill it in and apply it in one line:

```
gnss_band=5 <LO MHz> <rate MS/s> [bandwidth MHz] [txAtt dB] [rxGain dB]
gnss_band=5 1575.42 30.72 18 70 40         L1, 30.72 MS/s, 18 MHz wide, TX att 70 dB, RX gain 40 dB
gnss_band=5 1590 40                        only LO and rate: bandwidth automatic, gain / attenuation unchanged
gnss_band=5 1590 40 0 70 40                bandwidth 0 = automatic, then attenuation and gain
gnss_band=5                                applies what was stored last
```

The values are checked against the AD9361 range first (a bad value changes nothing) and
kept until power-off. `gnss_band?` shows what is stored. Unlike `radio=`, profile 5 is a
real profile: the radio guard keeps it (puts it back if a GUI changes the radio) until
you choose another `gnss_band=N`. Units are recognised by size, as in `radio=`
(next section). Attenuation and gain are optional; left out, they stay as they are.

## Every radio value by hand: `radio=` (firmware 4.2.0)

One command sets **LO, sample rate, RF bandwidth, TX attenuation and RX gain**.
Type it on one line, values separated by blanks (commas also work):

```
radio=<rxLO> <txLO> <rxRate> <txRate> <txAtt dB> <rxGain dB> [rfBandwidth]
radio=<LO> <rate> <txAtt dB> <rxGain dB> [rfBandwidth]          same LO and rate on RX and TX
radio?                                                         show what is set now
```

Examples:

```
radio=1575.42 1575.42 30.72 30.72 70 40 18       L1 only, 30.72 MS/s, 18 MHz wide, TX att 70 dB, RX gain 40 dB
radio=1582.5 61.44 70 40                         whole GNSS block, bandwidth chosen automatically
radio=1575420000 1575420000 30720000 30720000 70.25 35 18000000      the same thing in Hz
```

| value | unit | recognised by size | range |
|---|---|---|---|
| LO | MHz with any decimals (1575.42, 1568.256) or Hz | below 100000 = MHz | 70 to 6000 MHz |
| sample rate | MS/s with any decimals (30.72) or Hz | below 1000 = MS/s | 2.083333 to 61.44 MS/s |
| RF bandwidth | MHz or Hz | below 1000 = MHz | 0.2 to 56 MHz (left out: 91 percent of the rate, at most 56 MHz) |
| TX attenuation | dB, 0.25 steps | | 0 to 89.75 dB (smaller = louder) |
| RX gain | dB | | 1 to 76 dB (both channels, manual, also the auto reference) |

* A value outside the range is refused with a message and **nothing is changed**.
* RX and TX share **one** sample rate inside the AD9361 (FDD): give the same value
  twice; different values are refused.
* TX attenuation below 30 dB prints a warning (loud: cable and attenuator only).
* The AD9361 FIRs are switched off (a band-pass FIR would remove most of the band),
  the DC / quadrature calibrations are redone, and the noise floors are learned
  again. The radio guard then stays off until `gnss_band=N`.
* After the change the console prints the table of `radio?`, including which
  GNSS signals the new LO / rate / bandwidth still carries.

Single values, also with decimals / units (each keeps the radio guard off):

```
rx_lo_freq=1575.42      tx_lo_freq=1575.42       MHz with decimals (or Hz)
rx_samp_freq=30.72      tx_samp_freq=30.72       MS/s (or Hz)
rx_rf_bandwidth=18      tx_rf_bandwidth=18       MHz (or Hz)
tx_att_db=70.5          TX1 attenuation in dB    (tx1_attenuation= in milli-dB still works)
gnss_rx_gain=40         both RX gains in dB
```

Long lines now work (the old console buffer held 30 characters; it is 128 now)
and up to 10 values per command.

## Changing the LO and the sample rate by hand (since firmware 4.1.6)

A value typed with `rx_lo_freq=`, `tx_lo_freq=`, `rx_samp_freq=`, `tx_samp_freq=`,
`rx_fir_en=` or `tx_fir_en=` (or `radio=`) is **kept**; the console says "radio set
by hand: kept". Before 4.1.6 the radio guard put the `gnss_band=` profile back
about a second later ("radio changed outside gnss_band= ... restored"). The guard
stays off until you type `gnss_band=N`, which sets a profile again and switches the
guard back on. If RX and TX end up different, it warns once. A change made by a GUI
or another program that does not use these commands is still put back.
`gnss_band?` shows "set by hand" while the guard is off.
AD9361 limits: sample rates below about 2.08 MS/s are refused ("Failed to find
suitable dividers"); the vendor LO command took whole MHz only before 4.2.0.
