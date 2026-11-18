# e310lab: E310 + GNSS receiver test logger (11 Nov 2026, v1.1)

Package e310_gnss_lab.zip. Python 3.8+, pyserial and matplotlib. Runs on Windows and Linux.

## v1.1: PuTTY stays the console
- **Start:** `python e310lab.py` with no arguments, or `run_windows.bat` / `run_linux.sh`.
  - It lists the serial ports and asks for the setup, remembering each answer in e310lab_config.json.
  - Questions: E310 port and baud; the TCP port PuTTY connects to (5001); GNSS port, baud and type; GNSS viewer TCP port (5000) or virtual COM; KPI period (2 s); live status period (10 s); test name; output folder; optional plan.
- **PuTTY bridge:** e310lab owns the E310 COM port. PuTTY connects to 127.0.0.1:5001 with connection type Raw, Local echo Force off and Local line editing Force off.
  - Traffic is passed both ways and logged; commands typed in PuTTY are logged with source `putty`.
  - Telnet negotiation bytes are dropped.
  - The hidden `gnss_kpi?` polls are only sent when the prompt is idle and nobody has typed for 1.5 s, and their replies are not forwarded to PuTTY.
- **e310lab window:**
  - Every 10 s it prints a live status: receiver fix, used and tracked satellites, C/N0 per system, top-4 C/N0, and the E310 mode, J/N, suppression, gain, noise floor and output.
  - Commands: status / Enter, sats, watch, mark, seg, end, restart, ttff, e310 <cmd>, run <plan>, plot, help.
  - `q` / Ctrl+C stops the test, saves everything and writes the report.
- **Plan prompts:** the plan waits for `go` (or Enter) in the e310lab window.
- **Unchanged from v1.0:**
  - NMEA parsing (GGA, GSA, GSV; GPS, GLONASS, Galileo, BeiDou, QZSS);
  - the GNSS viewer re-serve on TCP / virtual COM;
  - SkyTraq / u-blox restart and TTFF;
  - the report: segment table, TTFF, 5 plots, report.html.
- **Dry runs passed** with the simulated board and receiver:
  - a PuTTY-like TCP client: commands passed, polls hidden, IAC stripped;
  - the setup questions;
  - plan go-prompts;
  - the report.
