#!/usr/bin/env python3
"""
gnss_logger.py -- one PC logger for the two things you read by eye:
  * the E310 console (what PuTTY / your serial terminal shows), and
  * the GNSS receiver (what GNSS Viewer shows: the NMEA the Venus 8 sends on its COM port),
both with the PC clock on every line, in a NEW FOLDER EVERY HOUR:

    <out>/<YYYYMMDD>/H<hh>/e310_console.txt    every line of the E310 console, "[hh:mm:ss.mmm] text"
    <out>/<YYYYMMDD>/H<hh>/gnss_nmea.txt       every NMEA line of the receiver, "[hh:mm:ss.mmm] $GNGGA,..."
    <out>/<YYYYMMDD>/H<hh>/gnss_summary.csv    one row per second: fix, satellites used, HDOP, position, C/N0 per satellite
    <out>/<YYYYMMDD>/H<hh>/session.txt         what was started, with which ports

NOTE: a serial port can be opened by ONE program. Close GNSS Viewer (and your terminal) before starting this;
this logger then also works as the E310 terminal: what you type here goes to the E310.
(If you must keep GNSS Viewer open, use its menu DataLog to save its own file, and give --gnss none here.)

    pip install pyserial
    python gnss_logger.py --e310 COM20 --gnss COM11 --out C:\\gnss_logs
"""
import argparse, datetime, os, queue, sys, threading, time

try:
    import serial
except ImportError:
    serial = None

# ------------------------------------------------------------------ hourly folders
class Hourly:
    """Owns the output files; every call checks the hour and, when it changed, closes the old files and opens a
    new folder <out>/<YYYYMMDD>/H<hh>/ (the 'new folder every hour')."""
    CSV_HEAD = ("pc_time,nmea_utc,status,fix_quality,fix_mode,sats_used,sats_in_view,hdop,pdop,vdop,lat,lon,alt_m,"
                "speed_kn,n_gps,n_glo,n_gal,n_bds,n_qzss,n_tracked,cn0_mean,cn0_max,first_fix_s,sats(sys_prn:cn0:elev:az)\n")
    def __init__(self, out):
        self.out, self.key, self.files, self.dir = out, None, {}, None
        self.lock = threading.Lock()
    def _open(self, now):
        key = now.strftime("%Y%m%d/H%H")
        if key == self.key: return
        for f in self.files.values(): f.close()
        self.files = {}
        self.key = key
        self.dir = os.path.join(self.out, now.strftime("%Y%m%d"), now.strftime("H%H"))
        os.makedirs(self.dir, exist_ok=True)
    def write(self, name, text, now=None):
        now = now or datetime.datetime.now()
        with self.lock:
            self._open(now)
            f = self.files.get(name)
            if f is None:
                path = os.path.join(self.dir, name)
                new = not os.path.exists(path)
                f = open(path, "a", encoding="utf-8", newline="\n")
                self.files[name] = f
                if new and name == "gnss_summary.csv": f.write(self.CSV_HEAD)
            f.write(text); f.flush()
    def close(self):
        with self.lock:
            for f in self.files.values(): f.close()

def stamp(now): return now.strftime("[%H:%M:%S.") + "%03d]" % (now.microsecond // 1000)

# ------------------------------------------------------------------ NMEA
def nmea_ok(line):
    """True when the checksum is right (or absent)."""
    if not line.startswith("$"): return False
    if "*" not in line: return True
    body, _, cs = line[1:].partition("*")
    try: want = int(cs[:2], 16)
    except ValueError: return False
    x = 0
    for ch in body: x ^= ord(ch)
    return x == want

SYS = {"GP": "G", "GL": "R", "GA": "E", "GB": "C", "BD": "C", "GQ": "Q", "GI": "I", "GN": "N"}   # G=GPS R=GLONASS E=Galileo C=BeiDou Q=QZSS

def ddmm(v, hemi):
    if not v: return ""
    try:
        f = float(v); deg = int(f // 100); val = deg + (f - deg * 100) / 60.0
        return "%.7f" % (-val if hemi in ("S", "W") else val)
    except ValueError: return ""

class Gnss:
    """Collects GGA / GSA / GSV / RMC and writes one CSV row per RMC (about 1 Hz)."""
    def __init__(self, hourly):
        self.h = hourly; self.gga = {}; self.gsa = {}; self.sats = {}; self.gsv_tmp = {}
        self.t_start = time.time(); self.first_fix = None
    def line(self, text, now):
        if not nmea_ok(text): return
        f = text[1:].split("*")[0].split(",")
        tid = f[0]
        if len(tid) < 5: return
        talker, kind = tid[:-3], tid[-3:]
        if kind == "GGA":
            self.gga = dict(utc=f[1], lat=ddmm(f[2], f[3]), lon=ddmm(f[4], f[5]), q=f[6], used=f[7], hdop=f[8], alt=f[9])
            if self.first_fix is None and f[6] not in ("", "0"): self.first_fix = time.time() - self.t_start
        elif kind == "GSA":
            self.gsa = dict(mode=f[2], pdop=f[15] if len(f) > 15 else "", hdop=f[16] if len(f) > 16 else "", vdop=f[17] if len(f) > 17 else "")
        elif kind == "GSV":
            try: total, num = int(f[1]), int(f[2])
            except (ValueError, IndexError): return
            sysc = SYS.get(talker, "?")
            lst = self.gsv_tmp.setdefault(talker, [])
            if num == 1: lst.clear()
            inview = f[3]
            for i in range(4, len(f) - 3, 4):
                prn, el, az, snr = f[i:i + 4]
                if prn: lst.append((sysc, prn, snr, el, az))
            if num == total:
                self.sats[talker] = (inview, list(lst))
        elif kind == "RMC":
            self.row(f, now)
    def row(self, f, now):
        g, a = self.gga, self.gsa
        allsats = [s for _, (_, lst) in sorted(self.sats.items()) for s in lst]
        cn = [float(s[2]) for s in allsats if s[2] not in ("", "0")]
        cnt = lambda c: sum(1 for s in allsats if s[0] == c)
        inview = sum(int(v[0]) for v in self.sats.values() if v[0].isdigit())
        fixq = g.get("q", "")
        fs = "%.1f" % self.first_fix if self.first_fix is not None else ""
        spd = f[7] if len(f) > 7 else ""
        row = [now.strftime("%Y-%m-%d %H:%M:%S"), f[1] if len(f) > 1 else "", f[2] if len(f) > 2 else "", fixq, a.get("mode", ""),
               g.get("used", ""), str(inview), g.get("hdop", ""), a.get("pdop", ""), a.get("vdop", ""), g.get("lat", ""),
               g.get("lon", ""), g.get("alt", ""), spd, str(cnt("G")), str(cnt("R")), str(cnt("E")), str(cnt("C")),
               str(cnt("Q")), str(len(cn)), "%.1f" % (sum(cn) / len(cn)) if cn else "", "%.0f" % max(cn) if cn else "", fs,
               ";".join("%s%s:%s:%s:%s" % s for s in allsats)]
        self.h.write("gnss_summary.csv", ",".join(row) + "\n", now)

# ------------------------------------------------------------------ readers
def reader(port, name, handler, stop):
    buf = b""
    while not stop.is_set():
        try: data = port.read(512)
        except Exception as e:
            handler("# %s read error: %s" % (name, e), datetime.datetime.now()); time.sleep(1); continue
        if not data: continue
        buf += data
        while b"\n" in buf or b"\r" in buf:
            i = min(x for x in (buf.find(b"\n"), buf.find(b"\r")) if x >= 0)
            line, buf = buf[:i], buf[i + 1:]
            if line.strip(): handler(line.decode("ascii", "replace").rstrip(), datetime.datetime.now())

def main():
    ap = argparse.ArgumentParser(description="E310 console + GNSS receiver logger with a new folder every hour")
    ap.add_argument("--e310", required=True, help="E310 console port (the one PuTTY uses), e.g. COM20")
    ap.add_argument("--gnss", default="none", help="GNSS receiver NMEA port (the one GNSS Viewer uses), e.g. COM11, or none")
    ap.add_argument("--gnss-baud", type=int, default=115200)
    ap.add_argument("--e310-baud", type=int, default=115200)
    ap.add_argument("--out", default="gnss_logs")
    ap.add_argument("--poll", type=int, default=0, help="send gnss_report? to the E310 every N seconds (0 = rely on the firmware's own gnss_log)")
    ap.add_argument("--no-set-clock", action="store_true", help="do not send gnss_time= to the E310 at start")
    a = ap.parse_args()
    if serial is None: sys.exit("pyserial is missing: pip install pyserial")

    hourly = Hourly(a.out); gnss = Gnss(hourly); stop = threading.Event()
    e = serial.Serial(a.e310, a.e310_baud, timeout=0.2)
    g = serial.Serial(a.gnss, a.gnss_baud, timeout=0.2) if a.gnss.lower() != "none" else None
    t0 = datetime.datetime.now()
    hourly.write("session.txt", "%s started: e310=%s gnss=%s out=%s poll=%ds\n" % (t0.isoformat(" ", "seconds"), a.e310, a.gnss, a.out, a.poll), t0)

    def on_e310(line, now): hourly.write("e310_console.txt", stamp(now) + " " + line + "\n", now); print(stamp(now), line)
    def on_gnss(line, now):
        hourly.write("gnss_nmea.txt", stamp(now) + " " + line + "\n", now); gnss.line(line, now)
    threading.Thread(target=reader, args=(e, "e310", on_e310, stop), daemon=True).start()
    if g: threading.Thread(target=reader, args=(g, "gnss", on_gnss, stop), daemon=True).start()

    def send(cmd): e.write((cmd + "\r").encode("ascii")); hourly.write("e310_console.txt", stamp(datetime.datetime.now()) + " >>> " + cmd + "\n")
    if not a.no_set_clock:
        n = datetime.datetime.now(); time.sleep(0.5)
        send("gnss_time=%d %d %d %d %d %d" % (n.year, n.month, n.day, n.hour, n.minute, n.second))
    def poller():
        while not stop.wait(a.poll): send("gnss_report?")
    if a.poll > 0: threading.Thread(target=poller, daemon=True).start()
    print("logging to %s ; type E310 commands here, Ctrl+C to stop" % os.path.abspath(a.out))
    try:
        for cmd in sys.stdin:
            cmd = cmd.strip()
            if cmd: send(cmd)
    except KeyboardInterrupt:
        pass
    stop.set(); hourly.close()

if __name__ == "__main__":
    main()
