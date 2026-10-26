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
   gnss_pt: present at 0x43c00000, version 4.1
   SELFTEST: PASS ...
   ANTI-JAM (SMI-PI 4.1.3): ON, bypassed until a jammer appears.
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
