# Testing PL-NPI v2.5 on PuTTY, and what to send back

This guide sets PuTTY up so every line the board prints is saved to a file.
Then come the tests. Each one says what to type, what you should see, and
what to write down. At the end there is a **report template**: fill it in and
paste it into your next message, together with the `gnss_report` blocks.

> **Jammer and TX1 safety.** Run the jammer tests only on a **conducted**
> path (coax, attenuators, splitter/combiner, DC blocks where needed), in a
> **shielded enclosure**, or under a test authorisation that covers it.
> Never connect a jammer or the TX1 output to an antenna. Before you
> connect or disconnect anything on TX1, type `gnss_tx=0`. Keep enough
> fixed attenuation in front of the GNSS receiver: the AD9361 can deliver
> up to about +7 dBm with 0 dB attenuation.

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

Type commands exactly as shown and press Enter. `gnss_report` and
`gnss_report?` are the same command. By default the board prints only
jammer events, and they wait while you type. `gnss_aj_log=30` adds a status
line every 30 s. `gnss_aj_events=0` silences the console completely
(`gnss_report` still lists every event).

---

## 2. The lines to read

### `gnss_report`

The first line is always one of:

| First line | Meaning |
|---|---|
| `JAMMER STATUS: ON` | A jammer is present. Confirmed by 4 captures in a row (wideband) or by the CW scan (narrowband). It stays ON until every metric has been below the lower OFF thresholds for 1 s. |
| `JAMMER STATUS: OFF` | No jammer. Measurements are valid. |
| `JAMMER STATUS: UNKNOWN` | No valid measurement: no capture for > 0.5 s (DMA stopped, radio being changed), or anti-jam off (`gnss_npi=0`). |

Each `gnss_report` prints one snapshot from the last capture. It changes
nothing and measures nothing itself. Typing it twice in a row gives the
same block, apart from the `ms old` figure.

| Field | Meaning |
|---|---|
| `since`, `rule` | when the status last changed, which evidence did it, and the ON/OFF rules |
| `detection` | every metric next to its ON and OFF threshold: coherence, power J/N, RX gain cut, FPGA detector, CW scan |
| `cw scan` | each narrowband peak: RF frequency, level in both antennas, `CONFIRMED` or candidate |
| `filter` | whether PL-NPI is **in the TX1 path** and why. This is **separate** from the jammer status: a CW below the noise is notched while the PL-NPI filter stays bypassed. |
| `dc blocker`, `cw notches` | FPGA DC blocker on/off; notch frequencies in use (`[0] 1575.422 MHz ON`). With one CW both notches show the same frequency. |
| `rx2 equaliser` | `ON` with its 3 taps and how much deeper than one weight they null the jammer, or `off` (matched paths, a CW alone, or no jammer). It comes on by itself 0.5-1 s after a wideband jammer with unequal RX1/RX2 paths appears and goes off when the jammer is gone. |
| `notch placement` | confirms the notches are in front of the loop (bitstream 2.5) |
| `suppression`, `residual` | how much the jammer is reduced; residual 0 dB = fully removed |
| `gnss tracking` | the board cannot see the receiver's tracking state; read it in the receiver |
| `radio` | LO, sample rate, RX/TX RF bandwidth, `gnss_band=` |
| `rx gain` | RX1 and RX2 gain and the gain mode (AD9361 slow-attack AGC by default) |

### Events

| Line | Meaning |
|---|---|
| `[AJ 12.345 s]` | time since the anti-jam engine started (use it for latencies) |
| `TX1 ON` | the receiver starts getting a signal: start the stopwatch here |
| `CW at 1575.422 MHz, +22.2 dB in its FFT bin ...: notch 0 ON` | a CW was confirmed and notched |
| `*** JAMMER ON ***  J/N ..., WIDEBAND ...` or `... CW tone at RF ...` | status went ON. `detected by` says which evidence did it. |
| `NULLED 7 ms after detection` | first capture that saw the jammer removed |
| `jammer OFF after 4.5 s ...: JAMMER STATUS OFF` | status went OFF |

---

## 3. Tests

Use the **same receiver, antennas or simulator, cables and GNSS viewer** in
every test.

### T0. Boot check (every new BOOT.BIN)

1. Power on with PuTTY connected. Look for:
   ```
   gnss_pt: present at 0x43c00000, version 2.5
   ANTI-JAM (PL-NPI 2.5.0): ON, bypassed until a jammer appears.
   ```
2. Type `gnss_band?`. Check that it shows `4 = USER`, LO RX/TX 1570.250 MHz,
   50.00 MS/s, RF bandwidth 50.0 MHz, both FIRs `off`. GPS, Galileo/B1C and
   BeiDou B1I should be `OK`, and GLONASS `OUTSIDE`.
3. Type `gnss_report` **twice**. The first line of both must be
   `JAMMER STATUS: OFF` (no jammer connected), and the blocks must be the
   same. In `rx gain`, check `AD9361 slow-attack AGC`. In `dc blocker`, check
   `ON`; in `cw notches`, check `2 in the core`. Keep the blocks.

### T1. RF parameters check

Compare `gnss_report` → `radio`, `rx gain`, `tx` with your intended setup:
1570.25 MHz, 50/50, RX gain mode 2 starting at 40 dB, and TX attenuation
70 dB after `gnss_tx=1`. If something differs, paste the block. The radio
guard puts the profile back within 1 s if a vendor command changes the LO,
rate or FIRs. To change the profile, use `gnss_band=`.

### T2. Time measurements, no jammer

For each start type (cold, warm, hot, using the viewer's buttons), do three
runs **through the AntSDR** and three with the **antenna/simulator directly
on the receiver**:

1. `gnss_tx=0`, issue the start command in the viewer, then `gnss_tx=1`.
2. From the `TX1 ON` time, write down:
   - **acquisition**: first satellite tracked;
   - **first fix**: first 3D navigation solution;
   - the number of satellites used and the average C/N0 after 2 min.

If the AntSDR's times are within a few seconds of the direct ones, the board
adds nothing. A cold start cannot be much under 26–35 s on any receiver
(README_PL_NPI_v2.4.md §6). For < 25 s, use warm/hot start or assistance.

### T3. Calibrated CW jammer: J/S 10, 15, 30, 40 dB

Setup: GNSS simulator (known level, e.g. −130 dBm) and CW generator,
combined in a splitter/combiner, conducted into RX1 and RX2 (a 2-way splitter
after the combiner, equal cables).

1. **Calibrate:** J/S = P_jammer − P_GNSS at the same point (dB). Use the
   simulator's stated level. Take the combiner and cable losses into
   account; measure them with a spectrum analyser if possible. Write down
   the generator settings.
2. CW at L1 + a few kHz (e.g. 1575.422 MHz). For each J/S of 10, 15, 30
   and 40 dB, and each of `gnss_aj_notch=1` and `gnss_aj_notch=0`:
   1. Switch the jammer on. Note the PuTTY time of `CW at ...` and of
      `JAMMER ON`, which gives the **detection latency**.
   2. After 60 s: `gnss_report`, then satellites used, average C/N0 and
      fix (y/n) in the receiver.
   3. Switch the jammer off. Note the time of `JAMMER STATUS OFF`.
3. The board prints J/N, not J/S. For C/N0 45 dB-Hz in 50 MHz, J/N ≈ J/S −
   32 dB. Use this only as a cross-check; your calibration is the
   reference. At J/S 10 dB (J/N ≈ −22 dB) the CW is below what the scan
   detects, so `JAMMER STATUS: OFF` is expected there. In simulation it
   also costs no C/N0.

Expected from simulation (not yet measured): with the notch, C/N0 within
about 1.5 dB of the no-jammer value up to J/S 40 dB. Without it, about
−6 dB at J/S 30 and −13 dB at J/S 40.

### T4. Wideband jammer steps

Noise jammer (bandwidth ≥ 20 MHz, centred near 1575 MHz) at J/N 10, 20, 30,
40 dB as printed. At each step, write down: the `JAMMER ON` time,
`NULLED in`, suppression, residual, satellites, C/N0 and fix.

### T5. On/off cycles (tracking continuity and status stability)

`gnss_aj_csv=500`, then the jammer 30 s on / 30 s off, five times (CW at J/S
30 dB, then wideband at J/N 30 dB). Then `gnss_aj_csv=0`. Count in the log:

- **status changes**: `JAMMER ON` and `JAMMER STATUS OFF` lines. Expect
  exactly 5 + 5, with no alternation;
- **tracking continuity**: in the receiver, how many satellites lost lock,
  and whether the fix was kept through each switch.

Also type `gnss_report` several times during an ON phase. Every one must say
`JAMMER STATUS: ON`.

### T6. A/B comparisons (at the strongest jammer that still gives a fix)

| Type | Compares |
|---|---|
| `gnss_aj_notch=0` then `=1` | without / with the CW notches |
| `gnss_aj_dc=0` then `=1` | without / with the FPGA DC blocker (watch the `DC` figures in `powers`) |
| `gnss_rx_agc=2` then `=1` | firmware matched gain (v2.3 default) / AD9361 slow-attack AGC (v2.4 default) |
| `gnss_band=1` then `=4` | v2.3 profile (1582.5 MHz, 61.44 MS/s, with GLONASS) / USER 1570.25 MHz |
| `gnss_npi=0` then `=1` | no anti-jam at all / anti-jam |

After each change, wait 30 s, then `gnss_report` + satellites/C/N0.

### T7. Jammer already on at power-on

Jammer on (CW J/S 30 dB), then power the board on. `JAMMER ON` and the notch
should appear within the first second after the banner. Then `gnss_tx=1`
and measure time to fix.

### T8. Do the two RX ports really see different signals? (do this first)

A 2-antenna array can only null a jammer that arrives differently at the two
ports than the satellites do. If both ports see the same signal (one antenna
split in two, or a simulator and the jammer combined before a splitter) the
filter cancels the satellites too, and no spatial algorithm can lock.

1. Jammer **off**, TX1 on, receiver locked and showing C/N0 per satellite.
2. Type `gnss_aj_mode=3` (the filter always in and adapting). Wait 60 s.
3. Look at the satellites. With two separate antennas nothing changes (the
   weights stay near 0). If the C/N0 drops sharply or the satellites vanish,
   the ports are common: change the rig (two real antennas, or the jammer fed
   to the two ports through a splitter while the satellites come from the two
   antennas).
4. Type `gnss_aj_mode=0` to go back.

### T9. Cable mismatch and the RX2 equaliser (wideband jammer)

Wideband (noise) jammer at J/N 10, 20, 30 dB (the J/N the board prints), then
once with a deliberate extra 1-2 m of cable on RX2 (about 5 ns per metre).
For each, with `gnss_aj_eq=1` (default) and then `gnss_aj_eq=0`:

1. Wait 20 s after the jammer is on. `gnss_report`: note `rx2 equaliser`
   (ON or off), `suppression`, `residual`.
2. The receiver's C/N0 per satellite and whether it holds its fix.
3. `gnss_report` again with the jammer off after 10 s: `rx2 equaliser : off`.

Expected from simulation (not measured): with the extra cable the equaliser
switches on and the residual is clearly lower than with `gnss_aj_eq=0`.

---

## 4. Report template (copy, fill in, paste into your next message)

```
=== PL-NPI v2.5 TEST REPORT ===
Date:
Build: BOOT.BIN sha256 (Build/Logs/boot.log):           --defines:
Step 3 HLS: clock estimate ___ ns, II ___, PL_NPI_NOTCHES = 2 / 1 / 0     Step 6: BUILD_TIMING MET / VIOLATED
Receiver model / firmware:                      GNSS viewer:
Signal source (sky / simulator model, level dBm):
Jammer generator, combiner/splitter, losses measured? y/n:
Test path: conducted / shielded enclosure

T0  version 2.5? y/n   gnss_band=4 OK? y/n   two gnss_report identical? y/n   first line:
T1  radio/rx gain/tx as intended? y/n (paste if no)

T2                   acquisition [s]       first fix [s]        sats / avg C/N0
    AntSDR  cold     __ __ __              __ __ __             __ / __
            warm     __ __ __              __ __ __             __ / __
            hot      __ __ __              __ __ __             __ / __
    direct  cold     __ __ __              __ __ __             __ / __
            warm     __ __ __              __ __ __             __ / __
            hot      __ __ __              __ __ __             __ / __

T3  CW, J/S calibrated: P_GNSS ____ dBm at ________
    J/S  notch  printed J/N  CW-detect latency  STATUS ON latency  sats  avg C/N0  fix  STATUS OFF after
    10   on     ____         ____ ms            ____ ms            __    ____      y/n  ____ s
    10   off    ____         ____               ____               __    ____      y/n  ____
    15   on     ____         ____               ____               __    ____      y/n  ____
    15   off    ____         ____               ____               __    ____      y/n  ____
    30   on     ____         ____               ____               __    ____      y/n  ____
    30   off    ____         ____               ____               __    ____      y/n  ____
    40   on     ____         ____               ____               __    ____      y/n  ____
    40   off    ____         ____               ____               __    ____      y/n  ____

T4  WB J/N  JAMMER ON latency  NULLED in  suppression  residual  sats  avg C/N0  fix
    10      ____ ms            ____ ms    ____         ____      __    ____      y/n
    20      ____               ____       ____         ____      __    ____      y/n
    30      ____               ____       ____         ____      __    ____      y/n
    40      ____               ____       ____         ____      __    ____      y/n

T5  CW 30: ON lines __ OFF lines __, sats lost per switch __, fix kept y/n
    WB 30: ON lines __ OFF lines __, sats lost per switch __, fix kept y/n
    gnss_report during ON always "ON"? y/n
T6  notch 0/1: C/N0 __ / __   dc 0/1: C/N0 __ / __   agc 2/1: C/N0 __ / __, residual __ / __
    band 1/4: sats __ / __    npi 0/1: sats __ / __
T7  detected at boot? y/n, time to fix ___ s
T8  gnss_aj_mode=3 with no jammer: satellites unchanged? y/n  (n = ports are common)
T9  wideband, extra RX2 cable ___ m:  J/N  eq on/off  residual  C/N0 avg  fix
      eq=1     10/20/30   ___       ____      ____      y/n
      eq=0     10/20/30   ___       ____      ____      y/n

Anything strange (WARNING lines, "radio changed", DMA errors, UNKNOWN status, resets):

Paste below: two gnss_report blocks (one jammer OFF, one worst case ON) and
the JAMMER ON / NULLED / OFF and CW lines.
=== END ===
```

The PuTTY log files in `C:\AJlogs\` contain everything. If a test goes
wrong, send the whole log of that session.

---

## Changing the LO and the sample rate by hand (firmware 2.5.1)

`rx_lo_freq=` and `tx_lo_freq=` take **whole MHz**, `rx_samp_freq=` and
`tx_samp_freq=` take **Hz** (`rx_rf_bandwidth=` / `tx_rf_bandwidth=` too).
Set **both** RX and TX to the same value:

```
rx_lo_freq=1575
tx_lo_freq=1575
rx_samp_freq=30720000
tx_samp_freq=30720000
gnss_status?
```

Before this version the radio guard put the `gnss_band=` profile back about a
second later ("radio changed outside gnss_band= ... restored"). Now a value typed
with `rx_lo_freq=`, `tx_lo_freq=`, `rx_samp_freq=`, `tx_samp_freq=`,
`rx_fir_en=` or `tx_fir_en=` is **kept**; the console says "radio set by hand:
kept". The guard stays off until you type `gnss_band=N`, which sets a profile
again and switches the guard back on. If RX and TX end up different, it warns
once. A change made by a GUI or another program that does not use these commands
is still put back. `gnss_band?` shows "set by hand" while the guard is off. The
noise floors are learned again after each such change.
AD9361 limits: sample rates below about 2.1 MS/s are refused ("Failed to find
suitable dividers"); the LO takes only whole MHz here (use a profile in
`bands[]` for 1570.25 MHz, see the build notes).
