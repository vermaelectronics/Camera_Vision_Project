#!/usr/bin/env python3
"""
e310_gnss_logger.py  --  log the E310 console AND the SkyTraq receiver together,
on one clock, so every console change can be matched to what the satellites did.

Works on Windows (COM5, COM7 ...) and Linux (/dev/ttyUSB0, /dev/ttyACM0 ...) and with
any firmware (MOD-32, the LVDS app, ...): it only sends what you type.

It REPLACES PuTTY while it runs (a port can only be opened by one program):
close PuTTY first. You type E310 commands in this window exactly as in PuTTY.

GNSS Viewer AT THE SAME TIME (--share): the logger owns the SkyTraq port and
copies every byte to a virtual serial port that GNSS Viewer opens instead;
whatever GNSS Viewer sends (cold start, settings) goes back to the receiver.
    Windows: install com0com (free, signed: "com0com-3.0.0.0-i386-and-x64-signed"),
             it makes a pair, e.g. COM20 <-> COM21.
             python e310_gnss_logger.py --e310 COM5 --gnss COM7 --share COM20
             GNSS Viewer: connect to COM21, same baud as the receiver.
    Linux:   --share pty   makes a pseudo-terminal and prints its name
             (e.g. /dev/pts/4); open that in your viewer (or gpsmon / cgps).
A cold start sent from GNSS Viewer is recognised and timed like !cold.

    python e310_gnss_logger.py                       (asks for the two ports)
    python e310_gnss_logger.py --e310 COM5 --gnss COM7 --name barrage_5dB
    python3 e310_gnss_logger.py --e310 /dev/ttyUSB1 --gnss /dev/ttyUSB0 --kpi 10
(Linux: if the port refuses to open, add yourself to the dialout group:
 sudo usermod -aG dialout $USER, then log out and in.)

Typed lines:
    gnss_floor=0        -> sent to the E310 (any console command)
    !note rooftop CW 25 dB on   -> a note in the logs (not sent); use it for jammer on/off
    !gnss               -> show the satellites now
    !watch 10           -> one status line every 10 s (fix, used, C/N0); !watch 0 = off
    !cold               -> COLD START the SkyTraq (clears its ephemeris/almanac/time) and
                           measure the time to first fix (TTFF) and to 4 / 8 / 12 satellites
    !ttff <text>        -> start a TTFF measurement by hand (e.g. type it the moment you
                           power on the E310 or switch the jammer on)
    !quit               -> stop, write the summary and the chart

Files (folder logs/YYYYMMDD_HHMMSS[_name]/):
    e310_console.log      every console line with time; your commands as ">>> cmd"
    gnss_nmea.log         every NMEA sentence with time
    gnss_1s.csv           once a second: fix, satellites used / in view, C/N0 per
                          constellation (mean of used, mean of the best 4, max), per-satellite list
    markers.csv           your commands, notes and the E310 events (JAMMER ON, RESULT, ...)
    kpi.csv               every gnss_kpi? line as columns (--kpi N sends it every N s)
    ttff.csv              every !cold / !ttff: time to first fix and to 4/8/12 satellites used
    summary_by_step.csv   per step (between two commands / notes): satellites and C/N0,
                          whole step and the last 60 s (settled)
    chart.png             C/N0 and satellites used vs time, commands marked (needs matplotlib)
Send the whole folder (zip it).

Needs: pip install pyserial   (matplotlib optional, for chart.png)
"""
import argparse
import csv
import datetime as dt
import os
import re
import sys
import threading
import time

try:
    import pty            # noqa: F401  (Linux --share pty)
    HAVE_PTY = True
except ImportError:
    HAVE_PTY = False

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
CONSTS = ["GPS", "GLO", "GAL", "BDS", "QZSS"]
LETTER = {"GPS": "G", "GLO": "R", "GAL": "E", "BDS": "C", "QZSS": "J"}   # RINEX letters
TALKER = {"GP": "GPS", "GL": "GLO", "GA": "GAL", "BD": "BDS", "GB": "BDS", "GQ": "QZSS", "QZ": "QZSS"}
SYSID = {"1": "GPS", "2": "GLO", "3": "GAL", "4": "BDS", "5": "QZSS"}
EVENT_RE = re.compile(r"\[EVENT\]|RESULT:|START:|WEAK|JAMMER (ON|OFF)|NULLED|AUTO-START|GNSS_TX: (ON|OFF)|"
                      r"OUTPUT CLIPPING|levelling to|RX gain \d+ dB|firmware MOD-", re.I)
STALE_S = 3.0


def now():
    return time.time()


def tstr(t):
    return dt.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def prn_const(prn):
    """Constellation from the PRN number when the sentence does not say (GN talker, old NMEA)."""
    if 1 <= prn <= 32:
        return "GPS"
    if 65 <= prn <= 96:
        return "GLO"
    if 193 <= prn <= 200:
        return "QZSS"
    if 201 <= prn <= 263 or 401 <= prn <= 463:
        return "BDS"
    if 301 <= prn <= 336:
        return "GAL"
    return None


class SharePort:
    """The second copy of the receiver stream for GNSS Viewer: a COM port of a
    com0com pair (Windows / Linux tty) or a pseudo-terminal (Linux 'pty')."""

    def __init__(self, name, baud):
        self.fd = None
        self.ser = None
        if name.lower() == "pty":
            if not HAVE_PTY or os.name == "nt":
                sys.exit("--share pty works on Linux only; on Windows use a com0com pair (COM20)")
            import tty
            master, slave = os.openpty()
            tty.setraw(master)
            tty.setraw(slave)
            self.fd = master
            self._slave = slave                  # keep it open: the viewer can come and go
            self.name = os.ttyname(slave)
        else:
            self.ser = serial.Serial(name, baud, timeout=0.1, write_timeout=0)
            self.name = name

    def write(self, b):
        try:
            if self.fd is not None:
                os.write(self.fd, b)
            else:
                self.ser.write(b)
        except Exception:
            pass                                 # viewer closed / buffer full: drop, never block

    def read(self):
        try:
            if self.fd is not None:
                import select
                r, _, _ = select.select([self.fd], [], [], 0.1)
                return os.read(self.fd, 512) if r else b""
            return self.ser.read(512)
        except Exception:
            time.sleep(0.1)
            return b""

    def close(self):
        try:
            if self.ser:
                self.ser.close()
            if self.fd is not None:
                os.close(self.fd)
                os.close(self._slave)
        except Exception:
            pass


class GnssState:
    """Satellites from GSV (C/N0), GSA (used) and GGA (fix)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.sv = {}        # (const, prn) -> (cn0, t)
        self.used = {}      # const -> (set(prn), t)
        self.fix = 0
        self.nused = 0
        self.hdop = None
        self.utc = ""
        self.t_gga = 0.0
        self.n_sentences = 0

    @staticmethod
    def _checksum_ok(s):
        if "*" not in s:
            return True
        body, cs = s[1:].split("*", 1)
        c = 0
        for ch in body:
            c ^= ord(ch)
        try:
            return c == int(cs[:2], 16)
        except ValueError:
            return False

    def feed(self, line, t):
        if not line.startswith("$") or len(line) < 7 or not self._checksum_ok(line):
            return
        f = line.split("*")[0].split(",")
        talker, typ = f[0][1:3], f[0][3:]
        with self.lock:
            self.n_sentences += 1
            if typ == "GSV":
                self._gsv(talker, f, t)
            elif typ == "GSA":
                self._gsa(talker, f, t)
            elif typ == "GGA":
                self._gga(f, t)

    def _gsv(self, talker, f, t):
        const = TALKER.get(talker)
        rest = f[4:]
        if len(rest) % 4 == 1:          # NMEA 4.1 signal id at the end
            rest = rest[:-1]
        for i in range(0, len(rest) - 3, 4):
            try:
                prn = int(rest[i])
            except ValueError:
                continue
            c = const or prn_const(prn)
            if not c:
                continue
            try:
                cn0 = float(rest[i + 3])
            except ValueError:
                cn0 = None       # in view, not tracked
            self.sv[(c, prn)] = (cn0, t)

    def _gsa(self, talker, f, t):
        if len(f) < 15:
            return
        prns = []
        for x in f[3:15]:
            try:
                prns.append(int(x))
            except ValueError:
                pass
        const = TALKER.get(talker)
        if not const and len(f) >= 19:
            const = SYSID.get(f[18].strip())
        groups = {}
        for p in prns:
            c = const or prn_const(p)
            if c:
                groups.setdefault(c, set()).add(p)
        for c, s in groups.items():
            old = self.used.get(c)
            if old and t - old[1] < 0.5:      # same epoch, more sentences
                s = s | old[0]
            self.used[c] = (s, t)

    def _gga(self, f, t):
        if len(f) < 9:
            return
        self.utc = f[1]
        try:
            self.fix = int(f[6] or 0)
        except ValueError:
            self.fix = 0
        try:
            self.nused = int(f[7] or 0)
        except ValueError:
            self.nused = 0
        try:
            self.hdop = float(f[8]) if f[8] else None
        except ValueError:
            self.hdop = None
        self.t_gga = t

    def snapshot(self, t):
        """Per-constellation figures now (stale entries dropped)."""
        with self.lock:
            self.sv = {k: v for k, v in self.sv.items() if t - v[1] < STALE_S}
            used = {c: s for c, (s, tu) in self.used.items() if t - tu < STALE_S}
            out = {"utc": self.utc, "fix": self.fix if t - self.t_gga < STALE_S else 0,
                   "nused": self.nused if t - self.t_gga < STALE_S else 0, "hdop": self.hdop,
                   "consts": {}, "list": []}
            for c in CONSTS:
                sats = [(p, cn) for (cc, p), (cn, _) in self.sv.items() if cc == c]
                tracked = sorted([cn for _, cn in sats if cn], reverse=True)
                us = used.get(c)
                cn_used = [cn for p, cn in sats if cn and us and p in us]
                out["consts"][c] = {
                    "view": len(sats),
                    "tracked": len(tracked),
                    "used": len(us) if us is not None else None,
                    "cn0_used": sum(cn_used) / len(cn_used) if cn_used else None,
                    "cn0_top4": sum(tracked[:4]) / len(tracked[:4]) if tracked else None,
                    "cn0_max": tracked[0] if tracked else None,
                }
                for p, cn in sorted(sats):
                    if cn:
                        out["list"].append(f"{LETTER[c]}{p}:{int(cn)}{'*' if us and p in us else ''}")
            return out


class Logger:
    def __init__(self, a):
        self.a = a
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        if getattr(a, "name", ""):
            stamp += "_" + "".join(c for c in a.name if c.isalnum() or c in "-_.")
        self.dir = os.path.join(a.out, stamp)
        os.makedirs(self.dir, exist_ok=True)
        self.flock = threading.Lock()
        self.con_log = open(os.path.join(self.dir, "e310_console.log"), "w", encoding="utf-8", buffering=1)
        self.nmea_log = open(os.path.join(self.dir, "gnss_nmea.log"), "w", encoding="utf-8", buffering=1)
        self.mk_f = open(os.path.join(self.dir, "markers.csv"), "w", newline="", encoding="utf-8")
        self.mk = csv.writer(self.mk_f)
        self.mk.writerow(["time", "t_s", "type", "text"])
        self.g_f = open(os.path.join(self.dir, "gnss_1s.csv"), "w", newline="", encoding="utf-8")
        self.g = csv.writer(self.g_f)
        hdr = ["time", "t_s", "utc", "fix", "sats_used", "hdop"]
        for c in CONSTS:
            hdr += [f"{c}_view", f"{c}_tracked", f"{c}_used", f"{c}_cn0_used", f"{c}_cn0_top4", f"{c}_cn0_max"]
        self.g.writerow(hdr + ["sats (G GPS R GLONASS E Galileo C BeiDou J QZSS: C/N0, *=used)"])
        self.t0 = now()
        self.rows = []        # (t, snapshot) for the summary
        self.markers = []     # (t, type, text)
        self.kpi = []         # dicts
        self.state = GnssState()
        self.stop = threading.Event()
        self.e310 = self.gnss = None
        self.share = None
        self.watch_s = float(getattr(a, "watch", 0) or 0)

    # ------------------------------------------------------------- markers
    def marker(self, typ, text, t=None):
        t = t or now()
        with self.flock:
            self.markers.append((t, typ, text))
            self.mk.writerow([tstr(t), f"{t - self.t0:.1f}", typ, text])
            self.mk_f.flush()

    # ------------------------------------------------------------- E310
    def e310_reader(self):
        buf = b""
        while not self.stop.is_set():
            try:
                data = self.e310.read(512)
            except serial.SerialException as e:
                print(f"\n[logger] E310 port error: {e}")
                self.stop.set()
                break
            if not data:
                continue
            sys.stdout.write(data.decode("ascii", "replace"))
            sys.stdout.flush()
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.e310_line(ANSI.sub("", raw.rstrip(b"\r").decode("ascii", "replace")))

    def e310_line(self, line, t=None):
        t = t or now()
        with self.flock:
            self.con_log.write(f"[{tstr(t)}] {line}\n")
        s = line.strip()
        if s.startswith("KPI"):
            d = {"time": tstr(t), "t_s": f"{t - self.t0:.1f}"}
            for kv in s[3:].split():
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    d[k] = v
            self.kpi.append(d)
        elif EVENT_RE.search(s):
            self.marker("e310", s, t)

    def send(self, cmd):
        t = now()
        with self.flock:
            self.con_log.write(f"[{tstr(t)}] >>> {cmd}\n")
        self.marker("cmd", cmd, t)
        self.e310.write((cmd + "\r\n").encode("ascii", "replace"))

    def kpi_timer(self):
        while not self.stop.wait(self.a.kpi):
            try:
                self.e310.write(b"gnss_kpi?\r\n")
            except serial.SerialException:
                break

    # ------------------------------------------------------------- GNSS
    def open_gnss(self):
        bauds = [self.a.gnss_baud] + [b for b in (9600, 115200, 38400, 57600, 19200, 4800, 230400)
                                      if b != self.a.gnss_baud]
        for b in bauds:
            s = serial.Serial(self.a.gnss, b, timeout=0.2)
            t_end, good = now() + 2.5, 0
            while now() < t_end:
                ln = s.readline().decode("ascii", "replace").strip()
                if ln.startswith("$G") or ln.startswith("$B"):
                    good += 1
                    if good >= 3:
                        print(f"[logger] GNSS on {self.a.gnss} at {b} baud")
                        return s
            s.close()
        sys.exit(f"[logger] no NMEA on {self.a.gnss} at any baud: is GNSS Viewer still open, "
                 f"or is it the wrong port?")

    def gnss_cold(self):
        """SkyTraq binary 'System Restart' (ID 0x01), start mode 3 = cold. Payload 15 bytes:
        ID, mode, UTC year(2) month day hour min sec, lat(2) lon(2) alt(2) -- zero for cold."""
        payload = bytes([0x01, 0x03] + [0] * 13)
        cs = 0
        for b in payload:
            cs ^= b
        frame = b"\xa0\xa1" + len(payload).to_bytes(2, "big") + payload + bytes([cs]) + b"\r\n"
        self.gnss_ack = None
        self.gnss.write(frame)
        self.marker("cold", "SkyTraq cold start")
        print("[logger] SkyTraq COLD START sent; TTFF is being measured (see ttff.csv at !quit)")

    def gnss_reader(self):
        while not self.stop.is_set():
            try:
                raw = self.gnss.readline()
                if raw and self.share:
                    self.share.write(raw)
                if b"\xa0\xa1\x00\x02\x83\x01" in raw:
                    print("[logger] SkyTraq: cold start ACCEPTED (ACK)")
                    self.marker("gnss", "restart ACK")
                elif b"\xa0\xa1\x00\x02\x84\x01" in raw:
                    print("[logger] SkyTraq: cold start REFUSED (NACK)")
                    self.marker("gnss", "restart NACK")
                ln = raw.decode("ascii", "replace").strip()
            except serial.SerialException as e:
                print(f"\n[logger] GNSS port error: {e}")
                self.stop.set()
                break
            if not ln or "$" not in ln:
                continue
            ln = ln[ln.index("$"):]
            t = now()
            with self.flock:
                self.nmea_log.write(f"[{tstr(t)}] {ln}\n")
            self.state.feed(ln, t)

    def share_reader(self):
        """GNSS Viewer -> receiver. A SkyTraq restart (msg 0x01) is logged as a
        cold / warm / hot start so ttff.csv times it like !cold."""
        buf = b""
        while not self.stop.is_set():
            data = self.share.read()
            if not data:
                continue
            try:
                self.gnss.write(data)
            except serial.SerialException:
                break
            buf = (buf + data)[-256:]
            i = buf.find(b"\xa0\xa1")
            while i >= 0 and len(buf) >= i + 6:
                ln = int.from_bytes(buf[i + 2:i + 4], "big")
                if buf[i + 4] == 0x01 and ln >= 2:
                    mode = buf[i + 5]
                    kind = {1: "hot", 2: "warm", 3: "cold"}.get(mode, f"mode {mode}")
                    if mode == 3:
                        self.marker("cold", "SkyTraq cold start (from GNSS Viewer)")
                    else:
                        self.marker("ttff", f"SkyTraq {kind} start (from GNSS Viewer)")
                    print(f"\n[logger] GNSS Viewer sent a {kind} start: TTFF is being measured")
                buf = buf[i + 2:]
                i = buf.find(b"\xa0\xa1")

    def watch_tick(self):
        nxt = now()
        while not self.stop.is_set():
            time.sleep(0.5)
            if self.watch_s <= 0 or now() < nxt:
                continue
            nxt = now() + self.watch_s
            sn = self.state.snapshot(now())
            parts = []
            for c in CONSTS:
                k = sn["consts"][c]
                if k["view"]:
                    parts.append(f"{LETTER[c]} {k['used'] if k['used'] is not None else 0}/{k['tracked']}"
                                 f" {_f(k['cn0_top4']) or '-'}")
            print(f"\n[gnss {dt.datetime.now().strftime('%H:%M:%S')}] fix {sn['fix']} used {sn['nused']}"
                  f"  |  " + "  ".join(parts) + "   (used/tracked best-4 C/N0)")

    def gnss_tick(self):
        nxt = now() + 1.0
        while not self.stop.is_set():
            time.sleep(max(0.0, nxt - now()))
            nxt += 1.0
            t = now()
            sn = self.state.snapshot(t)
            self.rows.append((t, sn))
            row = [tstr(t), f"{t - self.t0:.1f}", sn["utc"], sn["fix"], sn["nused"],
                   "" if sn["hdop"] is None else sn["hdop"]]
            for c in CONSTS:
                k = sn["consts"][c]
                row += [k["view"], k["tracked"], "" if k["used"] is None else k["used"],
                        _f(k["cn0_used"]), _f(k["cn0_top4"]), _f(k["cn0_max"])]
            row.append(" ".join(sn["list"]))
            with self.flock:
                self.g.writerow(row)
                self.g_f.flush()

    def show_gnss(self):
        sn = self.state.snapshot(now())
        print(f"\n[gnss] fix {sn['fix']}  used {sn['nused']}  HDOP {sn['hdop']}")
        for c in CONSTS:
            k = sn["consts"][c]
            if k["view"]:
                print(f"   {c:4s} view {k['view']:2d} tracked {k['tracked']:2d} used {k['used']}  "
                      f"C/N0 used {_f(k['cn0_used']) or '-'}  best4 {_f(k['cn0_top4']) or '-'}  "
                      f"max {_f(k['cn0_max']) or '-'}")

    # ------------------------------------------------------------- end
    def finish(self):
        self.stop.set()
        time.sleep(0.3)
        if self.share:
            self.share.close()
        for p in (self.e310, self.gnss):
            try:
                p and p.close()
            except Exception:
                pass
        if self.kpi:
            keys = []
            for d in self.kpi:
                for k in d:
                    if k not in keys:
                        keys.append(k)
            with open(os.path.join(self.dir, "kpi.csv"), "w", newline="", encoding="utf-8") as fh:
                w = csv.DictWriter(fh, fieldnames=keys)
                w.writeheader()
                w.writerows(self.kpi)
        write_summary(self.dir, self.t0, self.rows, self.markers)
        write_ttff(self.dir, self.t0, self.rows, self.markers)
        make_chart(self.dir, self.t0, self.rows, self.markers)
        for fh in (self.con_log, self.nmea_log, self.mk_f, self.g_f):
            fh.close()
        print(f"\n[logger] saved in {self.dir}  -- zip the folder and send it.")


def _f(v):
    return "" if v is None else f"{v:.1f}"


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def write_summary(folder, t0, rows, markers):
    """One line per step: from each command / note to the next one."""
    bounds = [(t0, "start", "(logging started)")] + [(t, ty, tx) for t, ty, tx in markers
                                                      if ty in ("cmd", "note", "cold", "ttff")]
    t_end = rows[-1][0] if rows else now()
    out = []
    for i, (ts, ty, tx) in enumerate(bounds):
        te = bounds[i + 1][0] if i + 1 < len(bounds) else t_end
        seg = [sn for t, sn in rows if ts <= t < te]
        last = [sn for t, sn in rows if max(ts, te - 60.0) <= t < te]
        if not seg:
            continue
        r = {"start": tstr(ts), "t_s": f"{ts - t0:.0f}", "duration_s": f"{te - ts:.0f}", "type": ty, "step": tx,
             "fix_%": f"{100.0 * sum(1 for s in seg if s['fix'] > 0) / len(seg):.0f}",
             "sats_used_mean": _f(_mean([s["nused"] for s in seg])),
             "sats_used_last60s": _f(_mean([s["nused"] for s in last]))}
        for c in CONSTS:
            r[f"{c}_used_last60s"] = _f(_mean([s["consts"][c]["used"] for s in last]))
            r[f"{c}_cn0_used_mean"] = _f(_mean([s["consts"][c]["cn0_used"] for s in seg]))
            r[f"{c}_cn0_used_last60s"] = _f(_mean([s["consts"][c]["cn0_used"] for s in last]))
            r[f"{c}_cn0_top4_last60s"] = _f(_mean([s["consts"][c]["cn0_top4"] for s in last]))
        out.append(r)
    if out:
        with open(os.path.join(folder, "summary_by_step.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(out[0].keys()))
            w.writeheader()
            w.writerows(out)


def write_ttff(folder, t0, rows, markers):
    """Time to first fix after each !cold / !ttff: the fix must first be lost
    (cold) or absent, then come back; plus the time to 4 / 8 / 12 satellites used."""
    out = []
    starts = [(t, ty, tx) for t, ty, tx in markers if ty in ("cold", "ttff")]
    for i, (ts, ty, tx) in enumerate(starts):
        te = starts[i + 1][0] if i + 1 < len(starts) else float("inf")
        seg = [(t, sn) for t, sn in rows if ts + 0.5 <= t < te]
        if ty == "cold":                      # skip the last outputs before the restart took effect
            k = next((j for j, (t, sn) in enumerate(seg) if sn["fix"] == 0), None)
            seg = seg[k:] if k is not None else []
        r = {"start": tstr(ts), "t_s": f"{ts - t0:.0f}", "type": ty, "text": tx}
        first = next((t for t, sn in seg if sn["fix"] > 0), None)
        r["ttff_s"] = f"{first - ts:.0f}" if first else "no fix"
        for n in (4, 8, 12):
            tn = next((t for t, sn in seg if sn["nused"] >= n and sn["fix"] > 0), None)
            r[f"to_{n}_sats_s"] = f"{tn - ts:.0f}" if tn else "-"
        for c in CONSTS:
            tc = next((t for t, sn in seg if sn["fix"] > 0 and (sn["consts"][c]["used"] or 0) >= 1), None)
            r[f"first_{c}_s"] = f"{tc - ts:.0f}" if tc else "-"
        out.append(r)
        print(f"[logger] TTFF {tstr(ts)} ({tx}): first fix {r['ttff_s']} s, 4 sats {r['to_4_sats_s']} s, "
              f"8 sats {r['to_8_sats_s']} s, 12 sats {r['to_12_sats_s']} s")
    if out:
        with open(os.path.join(folder, "ttff.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(out[0].keys()))
            w.writeheader()
            w.writerows(out)


def make_chart(folder, t0, rows, markers):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("[logger] matplotlib not installed: no chart.png (pip install matplotlib)")
        return
    if len(rows) < 5:
        return
    t = [(r[0] - t0) / 60.0 for r in rows]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    for c in CONSTS:
        y = [r[1]["consts"][c]["cn0_top4"] for r in rows]
        if any(v is not None for v in y):
            a1.plot(t, [float("nan") if v is None else v for v in y], lw=1, label=f"{c} (best 4)")
    a2.plot(t, [r[1]["nused"] for r in rows], color="k", lw=1, label="satellites used")
    for tm, ty, tx in markers:
        if ty in ("cmd", "note", "cold", "ttff"):
            x = (tm - t0) / 60.0
            for ax in (a1, a2):
                ax.axvline(x, color={"cmd": "tab:red", "note": "tab:green"}.get(ty, "tab:purple"), lw=0.7 if ty in ("cmd", "note") else 1.5, alpha=0.6)
            a1.text(x, 1.0, " " + tx[:28], rotation=90, va="top", fontsize=7,
                    transform=a1.get_xaxis_transform())
    a1.set_ylabel("C/N0 dB-Hz")
    a1.legend(loc="lower left", fontsize=8)
    a1.grid(alpha=0.3)
    a2.set_ylabel("used")
    a2.set_xlabel("minutes (red = E310 command, green = note, purple = cold start / TTFF start)")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(folder, "chart.png"), dpi=120)
    plt.close(fig)


def ask_ports(a):
    """Missing ports: list what the PC has and ask (Windows COMx, Linux /dev/tty*)."""
    if a.e310 and a.gnss:
        return
    try:
        from serial.tools import list_ports
        ports = sorted(list_ports.comports(), key=lambda p: p.device)
    except Exception:
        ports = []
    print("\nSerial ports on this PC:")
    for i, p in enumerate(ports):
        print(f"  {i + 1}. {p.device:16s} {p.description}")
    if not ports:
        print("  (none listed -- type the port name, e.g. COM5 or /dev/ttyUSB0)")

    def pick(what, now):
        while not now:
            t = input(f"{what} port (number from the list or name): ").strip()
            if t.isdigit() and 1 <= int(t) <= len(ports):
                now = ports[int(t) - 1].device
            elif t:
                now = t
        return now
    a.e310 = pick("E310 console (PuTTY)", a.e310)
    a.gnss = pick("SkyTraq GNSS receiver (GNSS Viewer)", a.gnss)
    if not a.name:
        a.name = input("Test name for the folder (Enter = none): ").strip().replace(" ", "_")
    if not a.share:
        t = input("Also show the receiver in GNSS Viewer? virtual port for it "
                  "(com0com end, e.g. COM20; Linux: pty; Enter = no): ").strip()
        a.share = t
    if a.watch == 0:
        t = input("Print a satellite status line every N s (Enter = 10, 0 = off): ").strip()
        try:
            a.watch = float(t) if t else 10.0
        except ValueError:
            a.watch = 10.0
    if a.kpi == 0:
        t = input("Send gnss_kpi? every N seconds (Enter = never; only if your firmware has it): ").strip()
        try:
            a.kpi = float(t) if t else 0.0
        except ValueError:
            a.kpi = 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--e310", help="E310 console port (the one PuTTY uses), e.g. COM5 or /dev/ttyUSB1")
    ap.add_argument("--e310-baud", type=int, default=115200)
    ap.add_argument("--gnss", help="SkyTraq port (the one GNSS Viewer uses), e.g. COM7 or /dev/ttyUSB0")
    ap.add_argument("--gnss-baud", type=int, default=115200, help="tried first; others are tried if no NMEA")
    ap.add_argument("--kpi", type=float, default=0, help="send gnss_kpi? every N s (0 = off)")
    ap.add_argument("--out", default="logs")
    ap.add_argument("--name", default="", help="text added to the log folder name, e.g. barrage_5dB")
    ap.add_argument("--share", default="", help="copy the receiver to this port for GNSS Viewer: one end of "
                    "a com0com pair (COM20; the viewer opens COM21) or 'pty' on Linux")
    ap.add_argument("--watch", type=float, default=0, help="print a satellite status line every N s")
    a = ap.parse_args()
    ask_ports(a)
    if os.name == "nt":
        os.system("")          # Windows 10+: let the console show the E310 colour codes

    L = Logger(a)
    try:
        L.e310 = serial.Serial(a.e310, a.e310_baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {a.e310}: {e}  (close PuTTY first)")
    try:
        L.gnss = L.open_gnss()
    except serial.SerialException as e:
        sys.exit(f"cannot open {a.gnss}: {e}  (close GNSS Viewer first)")

    if a.share:
        try:
            L.share = SharePort(a.share, L.gnss.baudrate)
        except serial.SerialException as e:
            sys.exit(f"cannot open the share port {a.share}: {e}")
        print(f"[logger] receiver copied to {L.share.name} at {L.gnss.baudrate} baud: "
              + ("open the OTHER end of the com0com pair in GNSS Viewer, same baud"
                 if L.share.ser else f"open {L.share.name} in your viewer"))
    print(f"[logger] logging to {L.dir}")
    print("[logger] type E310 commands as in PuTTY;  !note <text>  !gnss  !watch N  !cold  !ttff  !quit\n")
    L.marker("note", "logging started")
    threads = [threading.Thread(target=f, daemon=True)
               for f in (L.e310_reader, L.gnss_reader, L.gnss_tick, L.watch_tick)]
    if L.share:
        threads.append(threading.Thread(target=L.share_reader, daemon=True))
    if a.kpi > 0:
        threads.append(threading.Thread(target=L.kpi_timer, daemon=True))
    for th in threads:
        th.start()
    L.e310.write(b"\r\n")
    try:
        while not L.stop.is_set():
            line = input()
            s = line.strip()
            w = s.split()[0] if s else ""
            if len(w) > 1 and w.endswith("!") and not w.startswith("!"):
                s = "!" + w[:-1] + s[len(w):]           # "gnss!" / "cold!" = "!gnss" / "!cold"
            # logger commands may also be typed with the ! at the end: gnss!, cold!, note! text, quit!
            tok = s.split(" ", 1)
            if tok and tok[0].endswith("!") and not tok[0].startswith("!") and \
                    tok[0][:-1].lower() in ("gnss", "cold", "ttff", "note", "quit", "q", "exit", "help", "watch"):
                s = "!" + tok[0][:-1].lower() + ((" " + tok[1]) if len(tok) > 1 else "")
                line = s
            if s in ("!help", "!?"):
                print("[logger] !note <text>  !gnss  !watch N  !cold  !ttff <text>  !quit   (also gnss!, cold!, note! ...)")
                continue
            if s in ("!quit", "!q", "!exit"):
                break
            if s.startswith("!note"):
                L.marker("note", s[5:].strip() or "note")
                print(f"[logger] note saved: {s[5:].strip()}")
            elif s == "!gnss":
                L.show_gnss()
            elif s.startswith("!watch"):
                try:
                    L.watch_s = float(s[6:].strip() or 10)
                except ValueError:
                    L.watch_s = 10.0
                print(f"[logger] status line every {L.watch_s:g} s" if L.watch_s > 0 else "[logger] status line off")
            elif s == "!cold":
                L.gnss_cold()
            elif s.startswith("!ttff"):
                L.marker("ttff", s[5:].strip() or "TTFF start")
                print("[logger] TTFF timer started")
            elif s.startswith("!"):                     # never send logger commands to the E310
                print("[logger] logger commands: !note <text>  !gnss  !cold  !ttff <text>  !quit "
                      "(the ! may also go at the end: gnss!)")
            else:
                L.send(line.rstrip("\r\n"))
    except (KeyboardInterrupt, EOFError):
        pass
    L.finish()


if __name__ == "__main__":
    main()
