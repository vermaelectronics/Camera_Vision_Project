# e310lab: E310 anti-jam test logger (Windows and Linux)

You keep driving the ANTSDR E310 from **PuTTY** and watching the sky in the **GNSS viewer**. e310lab sits in between and records everything on one time base:

- **The E310 console**, every line in and out. PuTTY connects to e310lab instead of the COM port. Every 2 s, when you are not typing, e310lab also asks the board for its anti-jam figures with `gnss_kpi?` (J/N, suppression, RX gain, noise floor, output level, mode). You never see those requests in PuTTY.
- **The GNSS receiver**: fix, satellites used and tracked, and the C/N0 of every satellite (GPS, GLONASS, Galileo, BeiDou), every second. The stream is passed on to the GNSS viewer.
- **Live status in its own window** every 10 s: receiver fix, satellites, C/N0 per system, and the E310 mode, J/N, suppression and gain.
- **At the end, a report** with plots and tables. To share it, zip the session folder (for example, to send to Claude).

```
   PuTTY ──TCP 127.0.0.1:5001──┐                       ┌──TCP 127.0.0.1:5000── GNSS viewer
                               e310lab ── COM (E310 UART)
                                       ── COM (GNSS receiver)
```

---

## 1. Install (once)

| | Windows | Linux |
|---|---|---|
| Python | 3.8 or newer from python.org; tick **"Add python.exe to PATH"** | `sudo apt install python3 python3-pip` |
| Packages | double-click `run_windows.bat` (installs them) or `python -m pip install -r requirements.txt` | `./run_linux.sh` (installs them) or `python3 -m pip install -r requirements.txt` |
| Serial access | nothing | `sudo usermod -aG dialout $USER`, then log out and back in |

## 2. Start the test

1. **Close PuTTY and the GNSS viewer** if they have the COM ports open. Only one program can hold a COM port, and e310lab must hold both.
2. Start e310lab:
   - **Windows:** double-click `run_windows.bat`, or run `python e310lab.py`.
   - **Linux:** `./run_linux.sh`, or `python3 e310lab.py`.
3. It lists the serial ports and asks a few questions. Press **Enter** to keep the value in [brackets]; it remembers your answers for next time.

   | Question | What to answer |
   |---|---|
   | E310 console port | the number or name of the port PuTTY used (e.g. `COM5`, `/dev/ttyUSB0`) |
   | E310 baud | `115200` |
   | TCP port PuTTY connects to | `5001` |
   | GNSS receiver port | the port the GNSS viewer used (e.g. `COM7`, `/dev/ttyACM0`) |
   | GNSS receiver baud | the receiver's baud, usually `115200` (SkyTraq may be `9600`) |
   | Receiver type | `skytraq` (or `ublox` / `nmea`) |
   | GNSS viewer over TCP | `5000` |
   | GNSS viewer over a virtual COM pair | `none`, or the e310lab end of a pair (see step 4) |
   | Anti-jam figures every N s | `2` |
   | Live status every N s | `10` |
   | Test name | e.g. `barrage_0dB` (the session folder name) |
   | Save sessions in folder | `sessions` |
   | Test plan file | Enter for none (you drive the test by hand) |

4. Press **Enter** to START. The window shows `TEST RUNNING` and the session folder.

### Connect PuTTY to the E310 (through e310lab)

In PuTTY → **Session**:

- **Host Name:** `127.0.0.1`
- **Port:** `5001`
- **Connection type:** **Raw** (on newer PuTTY: "Other: Raw")

In PuTTY → **Terminal**:

- **Local echo:** **Force off**
- **Local line editing:** **Force off**

The board echoes your characters itself; with these left on, every character appears twice. Save the PuTTY session as `E310 via e310lab` so it's one click next time, then click **Open**.

You now have the normal `E310>` prompt. Type `stap=1`, `pass=1`, `gnss_report?`, `rx?` and the rest exactly as before. e310lab prints `PuTTY connected`. If PuTTY is closed, connect it again any time; logging continues.

### Connect the GNSS viewer

| Viewer | How |
|---|---|
| A viewer that can connect over TCP (u-center: Receiver → Connection → Network; gpsd; RTKNAVI; QGIS) | TCP client to `127.0.0.1:5000` |
| SkyTraq GNSS Viewer, or any COM-only viewer, on **Windows** | install **com0com** (free) and create a pair, e.g. `COM11 ↔ COM12`. Answer `COM11` to "virtual COM pair"; the viewer opens **COM12** at the receiver's baud |
| COM-only viewer on **Linux** | first run `socat -d -d pty,raw,echo=0,link=/tmp/gnss_a pty,raw,echo=0,link=/tmp/gnss_b`, answer `/tmp/gnss_a`, and the viewer opens `/tmp/gnss_b` |

The viewer can still send commands to the receiver (restart, configuration); they are passed through.

## 3. Check the GNSS status live

The e310lab window prints this every 10 s (the "live status" answer):

```
[18:42:10] GNSS: fix q1 used 18 tracked 24  GPS 9/8@41.2  GLO 5/4@39.8  Gal 6/4@38.9  Bei 4/2@36.1  top4 44.0 dBHz
          E310: mode SMI JAMMED  J/N 31.2 dB  supp 39.5 dB  RX gain 36.0 dB  floor -58.4 dBFS  out -24.0 dBFS
```

| Field | Meaning |
|---|---|
| `fix q1` | GGA fix quality: 0 = no fix, 1 = GNSS fix, 2 = DGNSS |
| `used 18 tracked 24` | satellites in the solution / satellites with a C/N0 |
| `GPS 9/8@41.2` | GPS tracked / used, mean C/N0 in dB-Hz (the same for GLONASS, Galileo, BeiDou) |
| `top4` | mean C/N0 of the 4 strongest satellites |
| `mode SMI JAMMED` | E310 null core (`SMI` = stap on, `BYPASS` = pass), jammer present or `CLEAR` |
| `J/N`, `supp` | jammer-to-noise at the ADC, and how much of it the null removes |
| `RX gain`, `floor`, `out` | AUTO RX gain, noise floor, and TX1 output level |

Commands you type **in the e310lab window**:

| Command | What it does |
|---|---|
| `status` (or just Enter) | the status above, now |
| `sats` | table of every satellite: system, PRN, elevation, azimuth, C/N0 |
| `watch 5` / `watch off` | status every 5 s / no automatic status |
| `mark <text>` | timeline marker, e.g. `mark jammer ON 56 MHz 0 dB`. It is drawn on every plot |
| `seg <label>` | start a named test segment; it becomes one row of the report table |
| `end` | end the segment (a new `seg` also ends the previous one) |
| `restart cold` (or `warm` / `hot`) | send a restart to the receiver and measure TTFF (SkyTraq, u-blox) |
| `ttff cold` | only the TTFF timer, when you restart the receiver from the GNSS viewer |
| `e310 <command>` | send one command to the E310 from here (normally use PuTTY) |
| `run plans/barrage_56mhz.json` | run a test plan |
| `plot` | make the report now; the test keeps running |
| `help` | the list |
| **`q`** (or `quit`, `stop`, Ctrl+C) | **STOP the test** |

## 4. Stop the test

Type **`q`** and Enter in the e310lab window, or press Ctrl+C. e310lab then:

1. closes the segment that is open;
2. saves every file;
3. makes the report and prints the segment table;
4. prints where `report/report.html` is. Open it in a browser.

PuTTY shows the connection as closed. To work on the board without e310lab, open PuTTY on the COM port again as before.

## 5. Example: the 56 MHz barrage test

| | In PuTTY | In the e310lab window | Jammer |
|---|---|---|---|
| 1 | `pass=1`, wait for a fix | `seg clean pass`, wait 2 min | OFF |
| 2 | `stap=1` (automatic start, about 2 s) | `seg clean stap`, wait 2 min | OFF |
| 3 | `stap=0` | `seg barrage pass`, wait 90 s (the threat) | **ON** 56 MHz 0 dB |
| 4 | `stap=1` | `seg barrage stap`, wait 3 min | ON |
| 5 | | `seg barrage stap cold`, then `restart cold`; wait for TTFF + 2 min | ON |
| 6 | `gnss_report?` | | ON |
| 7 | | `seg recovery`, wait 2 min | OFF |
| 8 | | **`q`** | |

You can also answer `plans/barrage_56mhz.json` to the "test plan" question. The plan then sends the E310 commands itself and waits for you at each jammer change: switch the jammer, then type `go` (or just Enter) in the e310lab window. `plans/quick_check.json` is a 5-minute check of `pass` against `stap` with no jammer.

## 6. What is saved

```
sessions/20261111_184200_barrage_0dB/
  e310_console.log     every console line with a timestamp; [putty] [poll] [plan] [e310] [mark]
  commands.csv         every command, and whether PuTTY, a plan or the poll sent it
  kpi.jsonl            every gnss_kpi? reading
  gnss_nmea.log        every NMEA sentence with a timestamp
  gnss_epochs.csv      per second: fix, used, tracked, HDOP, position, per-system tracked/used/C/N0
  gnss_sats.csv        per second and satellite: system, PRN, elevation, azimuth, C/N0, used
  markers.csv, ttff.csv, session.json, e310_raw.bin
  report/report.html   tables + plots (open in a browser)
  report/summary.md, segments.csv, kpi.csv
  report/1_satellites.png   tracked per system, used, fix quality (with your markers)
  report/2_cn0.png          C/N0 per system and top-4
  report/3_cn0_heatmap.png  C/N0 of every satellite over time
  report/4_antijam_kpi.png  J/N, suppression, RX gain, noise floor, output
  report/5_segments.png     per segment: top-4 C/N0 and satellites used
```

The segment table has one row per segment with these columns:

| Group | Columns |
|---|---|
| Timing | duration |
| E310 | mode |
| Fix | fix %, satellites used and tracked |
| Per system | tracked and C/N0 for GPS, GLONASS, Galileo and BeiDou |
| Overall | top-4 C/N0 |
| Mean E310 figures | J/N, suppression, gain, noise floor, output |
| Result | PASS/FAIL if a plan set limits |

To remake a report later, possibly on another PC, run:

```
python e310lab.py analyze sessions/20261111_184200_barrage_0dB
```

## 7. Other ways to start

```
python e310lab.py ports                     # list serial ports
python e310lab.py run --e310 COM5 --gnss COM7 --gnss-tcp 5000 --putty-port 5001 --name barrage_0dB
python e310lab.py run --simulate            # dry run, no hardware (PuTTY: 127.0.0.1:5001)
```

Answer `sim` to the first question for the same dry run from the questions. The simulated board accepts `simjam=30` (jammer on) and `stap=1`.

If you answer `none` to the E310 port, PuTTY keeps the COM port itself. e310lab then records only the receiver and your markers; the E310 lines and figures are not in the report.

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `could not open port` / `Access is denied` | PuTTY or the viewer still holds the COM port; close it and connect them through e310lab |
| PuTTY shows every character twice | Terminal → Local echo **Force off**, Local line editing **Force off** |
| PuTTY shows strange characters at connect | the connection type is Telnet; set it to **Raw** |
| No E310 figures in the status line | the board runs the older app; the MVDR2 firmware (app_ant_mvdr2) has `gnss_kpi?` and the `E310>` prompt |
| `GNSS: no NMEA epochs yet (... bytes received)` | wrong receiver baud |
| `GNSS: no NMEA epochs yet (0 bytes received)` | wrong receiver port |
| `used` stays 0 while satellites are tracked | normal before the first fix |
| Linux `Permission denied` | the `dialout` group (section 1) |
| Windows shows `[1;32m` codes in the e310lab window | old console; use Windows Terminal (PuTTY itself shows colours) |
