#!/usr/bin/env python3
"""
e310lab.py - ANTSDR E310 anti-jam test bench logger (Windows and Linux)

One program replaces PuTTY and sits next to the GNSS viewer:
  * E310 console  : you type commands here (same as PuTTY); every line in and
                    out is logged with a timestamp; gnss_kpi? is polled in the
                    background (hidden) so the anti-jam figures are recorded
  * GNSS receiver : NMEA is parsed (GGA, GSA, GSV, RMC: GPS, GLONASS, Galileo,
                    BeiDou, QZSS) into per-second epochs and per-satellite C/N0
  * the receiver stream can be re-served on TCP and/or a second serial port,
    so the GNSS viewer keeps showing the sky while this program records it
  * markers (":mark barrage 0 dB") cut the session into test segments
  * analyze: plots (satellites, C/N0, heat map, anti-jam KPIs), segment table,
    TTFF table, HTML report

Run "python e310lab.py -h" or read README.md.
"""
import argparse
import csv
import datetime as dt
import html
import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time
from pathlib import Path

VERSION = "1.1 (11 Nov 2026)"
PROMPT_RE = re.compile(r"E310>\s*$")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
KPI_RE = re.compile(r"^KPI\s+(.*)$")

CONST_NAMES = ["GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "SBAS", "NavIC", "other"]
TALKER = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo", "GB": "BeiDou", "BD": "BeiDou",
          "GQ": "QZSS", "QZ": "QZSS", "GI": "NavIC", "IR": "NavIC"}
# NMEA 4.10 GSA system id
GSA_SYS = {"1": "GPS", "2": "GLONASS", "3": "Galileo", "4": "BeiDou", "5": "QZSS", "6": "NavIC"}


def now():
    return time.time()


def iso(t):
    return dt.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def const_from_prn(prn):
    """GN talker: guess the system from the PRN numbering in use."""
    if 1 <= prn <= 32:
        return "GPS"
    if 33 <= prn <= 64 or 120 <= prn <= 158:
        return "SBAS"
    if 65 <= prn <= 96:
        return "GLONASS"
    if 193 <= prn <= 199:
        return "QZSS"
    if 201 <= prn <= 263 or 401 <= prn <= 463:
        return "BeiDou"
    if 301 <= prn <= 336:
        return "Galileo"
    return "other"


def tolerant_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


# =============================================================================
#  Serial helpers
# =============================================================================
def open_serial(port, baud):
    import serial  # pyserial
    return serial.Serial(port, baud, timeout=0.05, write_timeout=1)


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial is not installed: pip install pyserial")
        return
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"  {p.device:16s} {p.description}  [{p.hwid}]")
    print("\nE310 console: the Zynq UART (often a CP210x / FTDI 'USB-UART').")
    print("GNSS receiver: the receiver's own USB/UART port (the one the GNSS viewer uses).")


# =============================================================================
#  Session files
# =============================================================================
class Session:
    def __init__(self, root, name):
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / (f"{stamp}_{name}" if name else stamp)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.f_console = open(self.dir / "e310_console.log", "a", encoding="utf-8", newline="")
        self.f_raw = open(self.dir / "e310_raw.bin", "ab")
        self.f_nmea = open(self.dir / "gnss_nmea.log", "a", encoding="ascii", errors="replace", newline="")
        self.f_kpi = open(self.dir / "kpi.jsonl", "a", encoding="utf-8")
        self.w_cmd = self._csv("commands.csv", ["t", "time", "source", "command"])
        self.w_mark = self._csv("markers.csv", ["t", "time", "label"])
        self.w_ep = self._csv("gnss_epochs.csv", EPOCH_FIELDS)
        self.w_sat = self._csv("gnss_sats.csv", ["t", "time", "system", "prn", "elev", "az", "cn0", "used"])
        self.w_ttff = self._csv("ttff.csv", ["t", "time", "mode", "ttff_s", "result"])

    def _csv(self, name, fields):
        f = open(self.dir / name, "a", newline="", encoding="utf-8")
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(fields)
        return (f, w)

    def row(self, fw, values):
        f, w = fw
        with self.lock:
            w.writerow(values)
            f.flush()

    def console_line(self, t, src, text):
        with self.lock:
            self.f_console.write(f"{iso(t)} [{src}] {text}\n")
            self.f_console.flush()

    def raw(self, b):
        with self.lock:
            self.f_raw.write(b)

    def nmea(self, t, line):
        with self.lock:
            self.f_nmea.write(f"{t:.3f} {line}\n")

    def kpi(self, t, d):
        with self.lock:
            self.f_kpi.write(json.dumps({"t": t, **d}) + "\n")
            self.f_kpi.flush()

    def write_meta(self, meta):
        (self.dir / "session.json").write_text(json.dumps(meta, indent=2))

    def close(self):
        with self.lock:
            for f in (self.f_console, self.f_raw, self.f_nmea, self.f_kpi,
                      self.w_cmd[0], self.w_mark[0], self.w_ep[0], self.w_sat[0], self.w_ttff[0]):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass


EPOCH_FIELDS = (["t", "time", "utc", "fix_q", "fix_mode", "used", "hdop", "lat", "lon", "alt",
                 "tracked"] +
                [f"trk_{c}" for c in CONST_NAMES[:5]] +
                [f"used_{c}" for c in CONST_NAMES[:5]] +
                [f"cn0_{c}" for c in CONST_NAMES[:5]] +
                ["cn0_mean", "cn0_top4", "cn0_max"])


# =============================================================================
#  E310 console
# =============================================================================
class E310Console:
    """Owns the E310 serial port. The MVDR2 firmware prints the prompt
    'E310> ' after every command, which tells us when a command has finished.

    PuTTY bridge (putty_port > 0): PuTTY connects to 127.0.0.1:putty_port
    (connection type Raw) instead of opening the COM port itself. Every byte
    goes both ways unchanged and is logged; the hidden gnss_kpi? polls are not
    shown in PuTTY and only happen when you are not typing."""

    def __init__(self, port, session, show=True, putty_port=0):
        self.port = port
        self.s = session
        self.show = show
        self.buf = ""
        self.idle = threading.Event()        # prompt seen, nothing pending
        self.idle.set()
        self.owner = "user"                  # who sent the command in flight
        self.capture = None                  # list collecting the response lines
        self.done = threading.Event()
        self.send_lock = threading.Lock()
        self.wlock = threading.Lock()        # serial writes
        self.last_rx = now()
        self.last_key = 0.0
        self.typing = ""                     # PuTTY line being typed
        self.kpi_last = {}
        self.clients = []
        self.clock = threading.Lock()
        self.stop = False
        if putty_port:
            threading.Thread(target=self._tcp_server, args=(putty_port,), daemon=True).start()
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    # -- PuTTY bridge ---------------------------------------------------------
    def _tcp_server(self, tcp_port):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("127.0.0.1", tcp_port))
        except OSError as e:
            print(f"[e310lab] cannot open the PuTTY port {tcp_port}: {e}")
            return
        srv.listen(2)
        while not self.stop:
            try:
                c, a = srv.accept()
            except OSError:
                break
            c.settimeout(0.05)
            with self.clock:
                self.clients.append(c)
            print(f"\n[e310lab] PuTTY connected to the E310 console ({a[0]}:{a[1]})")
            threading.Thread(target=self._client_rx, args=(c,), daemon=True).start()

    def _client_rx(self, c):
        iac = 0
        while not self.stop:
            try:
                b = c.recv(256)
                if not b:
                    break
            except socket.timeout:
                continue
            except OSError:
                break
            data = bytearray()
            for x in b:                       # drop Telnet negotiation (if not Raw)
                if iac == 0:
                    if x == 0xFF:
                        iac = 1
                    elif x != 0x00:
                        data.append(x)
                elif iac == 1:                # after IAC
                    iac = 2 if x in (0xFB, 0xFC, 0xFD, 0xFE) else (3 if x == 0xFA else 0)
                elif iac == 2:                # option byte
                    iac = 0
                elif iac == 3:                # sub-negotiation until IAC SE
                    iac = 4 if x == 0xFF else 3
                else:
                    iac = 0 if x == 0xF0 else 3
            if not data:
                continue
            self.last_key = now()
            with self.send_lock:              # never in the middle of a poll
                with self.wlock:
                    self.port.write(bytes(data))
            for x in data:
                ch = chr(x)
                if ch in "\r\n":
                    if self.typing.strip():
                        t = now()
                        self.s.row(self.s.w_cmd, [f"{t:.3f}", iso(t), "putty", self.typing.strip()])
                        self.idle.clear()     # command running until the prompt
                    self.typing = ""
                elif x in (0x08, 0x7F):
                    self.typing = self.typing[:-1]
                elif 0x20 <= x < 0x7F:
                    self.typing += ch
        with self.clock:
            if c in self.clients:
                self.clients.remove(c)
        print("\n[e310lab] PuTTY disconnected (connect again any time)")

    def _to_clients(self, b):
        with self.clock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(b)
                except OSError:
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)

    # -- reader ---------------------------------------------------------------
    def _reader(self):
        while not self.stop:
            try:
                b = self.port.read(4096)
            except Exception as e:  # cable pulled
                print(f"\n[e310lab] E310 port read error: {e}")
                time.sleep(1)
                continue
            if not b:
                continue
            t = now()
            self.last_rx = t
            self.s.raw(b)
            if not (self.owner == "poll" and self.capture is not None):
                self._to_clients(b)
            text = b.decode("ascii", errors="replace").replace("\r\n", "\n").replace("\r", "\n")
            self.buf += text
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                self._line(t, line)
            clean = ANSI_RE.sub("", self.buf)
            if PROMPT_RE.search(clean):
                self.buf = ""
                self._prompt()

    def _line(self, t, line):
        clean = ANSI_RE.sub("", line).rstrip()
        if clean.startswith("E310>"):
            clean = clean[5:].lstrip()
            line = line.split("E310>", 1)[-1]
        src = self.owner if self.capture is not None else "e310"
        self.s.console_line(t, src, clean)
        m = KPI_RE.match(clean)
        if m:
            d = parse_kpi(m.group(1))
            self.kpi_last = d
            self.s.kpi(t, d)
        if self.capture is not None:
            self.capture.append(clean)
        if self.show and not (self.owner == "poll" and self.capture is not None):
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

    def _prompt(self):
        if self.capture is not None:
            self.done.set()
        self.idle.set()
        if self.show and self.owner != "poll":
            sys.stdout.write("E310> ")
            sys.stdout.flush()

    # -- send -----------------------------------------------------------------
    def send(self, cmd, owner="user", timeout=20.0, wait_prompt=True):
        """Send one command line; returns the response lines (prompt-delimited)."""
        with self.send_lock:
            if wait_prompt:
                self.idle.wait(2.0)
            self.owner = owner
            self.capture = []
            self.done.clear()
            self.idle.clear()
            self.s.row(self.s.w_cmd, [f"{now():.3f}", iso(now()), owner, cmd])
            with self.wlock:
                self.port.write((cmd + "\r").encode("ascii", errors="replace"))
                self.port.flush()
            ok = self.done.wait(timeout)
            out = self.capture or []
            self.capture = None
            if not ok:
                self.idle.set()      # no prompt (old firmware?): do not block forever
            self.owner = "user"
            return out

    def poll_kpi(self):
        """Hidden gnss_kpi? when the console is idle and nobody is typing."""
        if not self.idle.is_set() or now() - self.last_rx < 0.3:
            return None
        if self.typing or now() - self.last_key < 1.5:
            return None
        if not self.send_lock.acquire(blocking=False):
            return None
        self.send_lock.release()
        out = self.send("gnss_kpi?", owner="poll", timeout=3.0)
        for l in out:
            m = KPI_RE.match(l)
            if m:
                return parse_kpi(m.group(1))
        return None

    def close(self):
        self.stop = True


def parse_kpi(s):
    d = {}
    for tok in s.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            f = tolerant_float(v)
            d[k] = f if f is not None else v
    return d


# =============================================================================
#  GNSS receiver
# =============================================================================
class GnssReceiver:
    def __init__(self, port, session, rtype="nmea", tcp_port=0, forward=None):
        self.port = port
        self.s = session
        self.rtype = rtype
        self.stop = False
        self.lock = threading.Lock()
        self.sats = {}          # (system, prn, sig) -> dict(elev, az, cn0, t)
        self.used = {}          # system -> set(prn)
        self.gga = {}
        self.gsa_mode = None
        self.last_epoch = None
        self.fix_since = None
        self.ttff_wait = None   # (mode, t_start, timeout)
        self.clients = []
        self.forward = forward
        self.rx_bytes = 0
        if tcp_port:
            threading.Thread(target=self._tcp_server, args=(tcp_port,), daemon=True).start()
        if forward:
            threading.Thread(target=self._forward_reader, daemon=True).start()
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    # -- stream re-serve (GNSS viewer) ------------------------------------------
    def _tcp_server(self, tcp_port):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", tcp_port))
        srv.listen(4)
        print(f"[e310lab] GNSS stream re-served on TCP port {tcp_port}")
        while not self.stop:
            try:
                c, a = srv.accept()
            except OSError:
                break
            c.settimeout(0.05)
            with self.lock:
                self.clients.append(c)
            threading.Thread(target=self._tcp_client_rx, args=(c,), daemon=True).start()
            print(f"\n[e310lab] GNSS viewer connected from {a[0]}")

    def _tcp_client_rx(self, c):
        while not self.stop:          # viewer -> receiver (configuration commands)
            try:
                b = c.recv(1024)
                if not b:
                    break
                self.port.write(b)
            except socket.timeout:
                continue
            except OSError:
                break
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)

    def _forward_reader(self):
        while not self.stop:          # virtual COM pair -> receiver
            try:
                b = self.forward.read(1024)
                if b:
                    self.port.write(b)
            except Exception:
                time.sleep(0.2)

    def _fanout(self, b):
        with self.lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(b)
                except OSError:
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)
        if self.forward:
            try:
                self.forward.write(b)
            except Exception:
                pass

    # -- reader -------------------------------------------------------------------
    def _reader(self):
        buf = b""
        while not self.stop:
            try:
                b = self.port.read(4096)
            except Exception as e:
                print(f"\n[e310lab] GNSS port read error: {e}")
                time.sleep(1)
                continue
            if not b:
                continue
            self.rx_bytes += len(b)
            self._fanout(b)
            buf += b
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                raw, buf = buf[:i], buf[i + 1:]
                j = raw.rfind(b"$")       # binary messages share the stream
                if j < 0:
                    continue
                line = raw[j:].decode("ascii", errors="replace").strip()
                t = now()
                self.s.nmea(t, line)
                try:
                    self._nmea(t, line)
                except Exception:
                    pass
            if len(buf) > 65536:
                buf = buf[-4096:]

    def _nmea(self, t, line):
        if "*" in line:
            body, cs = line[1:].split("*", 1)
            c = 0
            for ch in body:
                c ^= ord(ch)
            if cs[:2].upper() != f"{c:02X}":
                return
        else:
            body = line[1:]
        f = body.split(",")
        talker, typ = f[0][:2], f[0][2:]
        if typ == "GSV":
            self._gsv(t, talker, f)
        elif typ == "GSA":
            self._gsa(talker, f)
        elif typ == "GGA":
            self._gga(t, f)
        elif typ == "RMC":
            pass

    def _gsv(self, t, talker, f):
        n = len(f) - 4
        sig = f[-1] if n % 4 == 1 else "1"
        nblk = n // 4
        with self.lock:
            for k in range(nblk):
                prn = tolerant_float(f[4 + 4 * k])
                if prn is None:
                    continue
                prn = int(prn)
                sysname = TALKER.get(talker) or const_from_prn(prn)
                cn0 = tolerant_float(f[7 + 4 * k])
                self.sats[(sysname, prn, sig)] = {
                    "elev": tolerant_float(f[5 + 4 * k]), "az": tolerant_float(f[6 + 4 * k]),
                    "cn0": cn0 or 0.0, "t": t}

    def _gsa(self, talker, f):
        sysid = f[18] if len(f) > 18 and f[18] else None
        prns = set()
        for x in f[3:15]:
            v = tolerant_float(x)
            if v is not None:
                prns.add(int(v))
        with self.lock:
            self.gsa_mode = f[2] if len(f) > 2 else None
            if sysid in GSA_SYS:
                self.used[GSA_SYS[sysid]] = prns
            elif talker in TALKER:
                self.used[TALKER[talker]] = prns
            else:   # GN without system id: split by PRN range
                for p in prns:
                    self.used.setdefault(const_from_prn(p), set()).add(p)

    def _gga(self, t, f):
        q = int(tolerant_float(f[6]) or 0)
        g = {"utc": f[1], "fix_q": q, "used": int(tolerant_float(f[7]) or 0),
             "hdop": tolerant_float(f[8]), "alt": tolerant_float(f[9]),
             "lat": nmea_deg(f[2], f[3]), "lon": nmea_deg(f[4], f[5])}
        self.gga = g
        self._epoch(t)
        if q > 0 and self.fix_since is None:
            self.fix_since = t
        if q == 0:
            self.fix_since = None
        if self.ttff_wait:
            mode, t0, tmo = self.ttff_wait
            if q > 0:
                ttff = t - t0
                self.s.row(self.s.w_ttff, [f"{t:.3f}", iso(t), mode, f"{ttff:.1f}", "FIX"])
                print(f"\n[e310lab] TTFF ({mode} start): {ttff:.1f} s")
                self.ttff_wait = None
            elif t - t0 > tmo:
                self.s.row(self.s.w_ttff, [f"{t:.3f}", iso(t), mode, "", f"NO FIX in {tmo:.0f} s"])
                print(f"\n[e310lab] TTFF ({mode} start): NO FIX within {tmo:.0f} s")
                self.ttff_wait = None

    def _epoch(self, t):
        with self.lock:
            # satellites not refreshed for 3 s are gone
            for k in [k for k, v in self.sats.items() if t - v["t"] > 3.0]:
                del self.sats[k]
            best = {}       # one entry per (system, prn): strongest signal
            for (sysn, prn, sig), v in self.sats.items():
                if (sysn, prn) not in best or v["cn0"] > best[(sysn, prn)]["cn0"]:
                    best[(sysn, prn)] = v
            used = {k: set(v) for k, v in self.used.items()}
        g = self.gga
        trk = {c: 0 for c in CONST_NAMES[:5]}
        usedn = {c: 0 for c in CONST_NAMES[:5]}
        cn = {c: [] for c in CONST_NAMES[:5]}
        allcn = []
        for (sysn, prn), v in best.items():
            u = prn in used.get(sysn, set())
            self.s.row(self.s.w_sat, [f"{t:.3f}", iso(t), sysn, prn, v["elev"], v["az"],
                                      v["cn0"], int(u)])
            if v["cn0"] > 0:
                allcn.append(v["cn0"])
                if sysn in trk:
                    trk[sysn] += 1
                    cn[sysn].append(v["cn0"])
            if u and sysn in usedn:
                usedn[sysn] += 1
        allcn.sort(reverse=True)
        mean = lambda a: (sum(a) / len(a)) if a else ""
        row = [f"{t:.3f}", iso(t), g.get("utc", ""), g.get("fix_q", 0), self.gsa_mode or "",
               g.get("used", 0), g.get("hdop", ""), g.get("lat", ""), g.get("lon", ""),
               g.get("alt", ""), len(allcn)]
        row += [trk[c] for c in CONST_NAMES[:5]]
        row += [usedn[c] for c in CONST_NAMES[:5]]
        row += [fmt1(mean(cn[c])) for c in CONST_NAMES[:5]]
        row += [fmt1(mean(allcn)), fmt1(mean(allcn[:4])), fmt1(allcn[0] if allcn else "")]
        self.s.row(self.s.w_ep, row)
        self.last_epoch = dict(zip(EPOCH_FIELDS, row))

    # -- receiver restart (TTFF) --------------------------------------------------
    def restart(self, mode, timeout=180.0):
        m = mode.lower()
        if self.rtype == "skytraq":
            code = {"hot": 1, "warm": 2, "cold": 3}[m]
            payload = bytes([0x01, code]) + bytes(13)
            pkt = b"\xA0\xA1" + len(payload).to_bytes(2, "big") + payload
            cs = 0
            for x in payload:
                cs ^= x
            pkt += bytes([cs]) + b"\r\n"
        elif self.rtype == "ublox":
            mask = {"hot": 0x0000, "warm": 0x0001, "cold": 0xFFFF}[m]
            payload = mask.to_bytes(2, "little") + bytes([0x02, 0x00])
            msg = bytes([0x06, 0x04]) + len(payload).to_bytes(2, "little") + payload
            a = b = 0
            for x in msg:
                a = (a + x) & 0xFF
                b = (b + a) & 0xFF
            pkt = b"\xB5\x62" + msg + bytes([a, b])
        else:
            print("[e310lab] restart needs --gnss-type skytraq or ublox; "
                  "restart the receiver in the GNSS viewer and type :ttff <mode>")
            return False
        self.port.write(pkt)
        self.fix_since = None
        self.ttff_wait = (m, now(), timeout)
        print(f"[e310lab] receiver {m} start sent; waiting up to {timeout:.0f} s for a fix")
        return True

    def ttff_manual(self, mode, timeout=180.0):
        self.fix_since = None
        self.ttff_wait = (mode, now(), timeout)

    def summary(self):
        e = self.last_epoch
        if not e:
            return f"GNSS: no NMEA epochs yet ({self.rx_bytes} bytes received)"
        parts = [f"fix q{e['fix_q']} used {e['used']} tracked {e['tracked']}"]
        for c in CONST_NAMES[:4]:
            parts.append(f"{c[:3]} {e['trk_' + c]}/{e['used_' + c]}@{e['cn0_' + c] or '-'}")
        parts.append(f"top4 {e['cn0_top4'] or '-'} dBHz")
        return "GNSS: " + "  ".join(parts)

    def sat_table(self):
        with self.lock:
            items = sorted(self.sats.items())
        print(f"  {'system':8s} {'prn':>4s} {'sig':>3s} {'el':>4s} {'az':>5s} {'C/N0':>5s}")
        for (sysn, prn, sig), v in items:
            print(f"  {sysn:8s} {prn:4d} {sig:>3s} {fmt0(v['elev']):>4s} {fmt0(v['az']):>5s} "
                  f"{fmt0(v['cn0']):>5s}")

    def close(self):
        self.stop = True


def nmea_deg(v, hemi):
    x = tolerant_float(v)
    if x is None:
        return ""
    d = int(x / 100)
    deg = d + (x - 100 * d) / 60.0
    return round(-deg if hemi in ("S", "W") else deg, 7)


def fmt1(x):
    return f"{x:.1f}" if isinstance(x, (int, float)) else x


def fmt0(x):
    return f"{x:.0f}" if isinstance(x, (int, float)) else "-"


# =============================================================================
#  Simulation (dry run without hardware)
# =============================================================================
class SimPort:
    """Minimal pyserial-like object: E310 console or NMEA receiver model."""

    def __init__(self, kind, world):
        self.kind = kind
        self.w = world
        self.out = b""
        self.lock = threading.Lock()
        self.tnext = now()
        if kind == "e310":
            self.out = b"\r\n=== ANTSDR E310 - GNSS anti-jam (SIMULATED) ===\r\nE310> "

    def read(self, n):
        time.sleep(0.02)
        if self.kind == "gnss" and now() >= self.tnext:
            self.tnext += 1.0
            with self.lock:
                self.out += self.w.nmea_epoch().encode()
        with self.lock:
            b, self.out = self.out[:n], self.out[n:]
        return b

    def write(self, b):
        if self.kind == "gnss":
            if b[:2] in (b"\xA0\xA1", b"\xB5\x62"):
                self.w.restart()
            return len(b)
        cmd = b.decode(errors="replace").strip()
        resp = self.w.e310(cmd)
        with self.lock:
            self.out += (cmd + "\r\n" + resp + "E310> ").encode()
        return len(b)

    def flush(self):
        pass

    def close(self):
        pass


class SimWorld:
    def __init__(self):
        self.jam = 0.0          # jammer J/N in dB at the antenna
        self.stap = 0
        self.t_fix = now() + 5
        import random
        self.r = random.Random(1)
        self.sky = [("GP", p) for p in (3, 7, 9, 14, 17, 21, 30)] + \
                   [("GL", p) for p in (65, 71, 72, 80)] + \
                   [("GA", p) for p in (2, 11, 25, 36)] + \
                   [("GB", p) for p in (6, 19, 22, 26, 33)]

    def restart(self):
        self.t_fix = now() + (35 if self.jam > 0 and self.stap else 25)

    def cn0_drop(self):
        if self.jam <= 0:
            return 0.0
        return 6.0 if self.stap else self.jam + 20

    def e310(self, cmd):
        c = cmd.lower()
        if c.startswith("simjam="):
            self.jam = float(c.split("=")[1])
            return f"[sim] jammer J/N {self.jam:.0f} dB\r\n"
        if c == "stap=1":
            self.stap = 1
            return "--- stap : anti-jam\r\n RESULT: MVDR RUNNING on TX1\r\n  MODE gnss_tx=1 pass=1 stap=1\r\n"
        if c in ("stap=0", "pass=1"):
            self.stap = 0
            return "  MODE gnss_tx=1 pass=1 stap=0\r\n"
        if c == "gnss_kpi?":
            jn = self.jam + self.r.uniform(-0.5, 0.5) if self.jam else 0
            supp = jn + 8 if (self.stap and self.jam) else 0
            gain = 52 - (16 if self.jam > 20 else 0)
            return (f"KPI mode={'SMI' if self.stap else 'BYPASS'} state={'JAMMED' if self.jam else 'CLEAR'} "
                    f"jn={jn:.1f} supp={supp:.1f} res=-8.0 nf=-58.2 gain={gain} agc=AUTO "
                    f"pk1=-6.0 pk2=-6.2 clip1=0 clip2=0 wn=2.1 out=-24.0 tx={1 if self.stap else 0}\r\n")
        return f"[sim] {cmd}: ok\r\n"

    def nmea_epoch(self):
        t = dt.datetime.now(dt.timezone.utc)
        hh = t.strftime("%H%M%S.00")
        fix = now() >= self.t_fix
        drop = self.cn0_drop()
        sats = []
        for tk, p in self.sky:
            cn0 = 44 - (p % 7) + self.r.uniform(-1, 1) - drop
            sats.append((tk, p, max(0, cn0)))
        used = [s for s in sats if s[2] > 30] if fix else []
        out = []
        q = 1 if (fix and len(used) >= 4) else 0
        out.append(nmea_wrap(f"GPGGA,{hh},2830.0000,N,07710.0000,E,{q},{len(used):02d},0.9,220.0,M,-35.0,M,,"))
        for tk in ("GP", "GL", "GA", "GB"):
            ss = [s for s in sats if s[0] == tk]
            for i in range(0, len(ss), 4):
                blk = ",".join(f"{p:02d},45,{(p * 37) % 360:03d},{c:.0f}" if c > 0 else f"{p:02d},45,{(p * 37) % 360:03d},"
                               for _, p, c in ss[i:i + 4])
                out.append(nmea_wrap(f"{tk}GSV,{(len(ss) + 3) // 4},{i // 4 + 1},{len(ss):02d},{blk}"))
            u = [f"{p:02d}" for t_, p, c in used if t_ == tk][:12]
            out.append(nmea_wrap(f"{tk}GSA,A,{3 if q else 1}," + ",".join(u + [""] * (12 - len(u))) + ",1.5,0.9,1.2"))
        return "".join(out)


def nmea_wrap(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"${body}*{c:02X}\r\n"


# =============================================================================
#  Test plan runner
# =============================================================================
class PlanRunner:
    """JSON plan: {"name": ..., "steps": [ {...}, ... ]}. Step keys:
       cmd      E310 command          wait     seconds after the step
       mark     marker label          prompt   operator instruction, Enter to go on
       measure  seconds (with label: a segment named label, marker at start and end)
       restart  hot|warm|cold (receiver; with ttff_timeout)
       expect   {"fix_pct_min":..,"used_min":..,"cn0_top4_min":..,"supp_min":..}
    """

    def __init__(self, lab):
        self.lab = lab

    def run(self, path, auto_yes=False):
        plan = json.loads(Path(path).read_text())
        lab = self.lab
        name = plan.get("name", Path(path).stem)
        lab.mark(f"PLAN START {name}")
        steps = plan.get("steps", [])
        for i, st in enumerate(steps, 1):
            if lab.quit:
                break
            print(f"\n[plan {i}/{len(steps)}] {json.dumps(st)}")
            if "prompt" in st:
                lab.mark(f"OPERATOR: {st['prompt']}")
                if auto_yes:
                    print(f"[plan] OPERATOR: {st['prompt']}  (auto-continue)")
                else:
                    lab.go.clear()
                    lab.waiting = True
                    print(f"\n>>> OPERATOR: {st['prompt']}\n>>> then type  go  (or just Enter) in THIS window")
                    while not lab.go.wait(0.5):
                        if lab.quit:
                            return
                    lab.waiting = False
            if "mark" in st:
                lab.mark(st["mark"])
            if "cmd" in st and lab.e310:
                lab.e310.send(st["cmd"], owner="plan", timeout=st.get("timeout", 30))
            if "restart" in st:
                if lab.gnss:
                    lab.mark(f"RESTART {st['restart']}")
                    lab.gnss.restart(st["restart"], st.get("ttff_timeout", 180))
                    t0 = now()
                    while lab.gnss.ttff_wait and now() - t0 < st.get("ttff_timeout", 180) + 2:
                        time.sleep(0.5)
            if "measure" in st:
                label = st.get("label", f"step {i}")
                lab.mark(f"SEG START {label}")
                if "expect" in st:
                    lab.expect[label] = st["expect"]
                self._sleep(st["measure"])
                lab.mark(f"SEG END {label}")
            if "wait" in st:
                self._sleep(st["wait"])
        lab.mark(f"PLAN END {name}")
        print("\n[plan] finished. :plot (or the analyze command) makes the report.")

    def _sleep(self, s):
        t_end = now() + s
        last = 0
        while now() < t_end and not self.lab.quit:
            if now() - last >= 10 and self.lab.gnss:
                print(f"[plan] {t_end - now():5.0f} s left | {self.lab.gnss.summary()}")
                last = now()
            time.sleep(0.25)


# =============================================================================
#  The bench
# =============================================================================
HELP = """
  Commands in THIS window (the E310 itself is driven from PuTTY):
    status | st          live receiver + anti-jam status now
    sats                 satellite table (system, PRN, elevation, azimuth, C/N0)
    watch <s> | watch off   status every s seconds (default 10)
    mark <text>          timeline marker, e.g.  mark barrage 56 MHz 0 dB ON
    seg <label>          start a named test segment (one row of the report table)
    end                  end the current segment
    restart cold|warm|hot  receiver restart + TTFF (SkyTraq / u-blox)
    ttff cold|warm|hot   TTFF timer only (restart done in the GNSS viewer)
    kpi <s> | kpi off    background gnss_kpi? period
    e310 <command>       send one command to the E310 from here
    run <plan.json>      run a test plan
    plot                 make the report of this session now (logging goes on)
    help                 this text
    quit | q | Ctrl+C    STOP the test: save everything and make the report
  (a leading ':' is accepted too, e.g. :mark)
"""


class Lab:
    def __init__(self, args):
        self.args = args
        self.quit = False
        self.expect = {}
        self.seg = None
        self.go = threading.Event()
        self.waiting = False
        name = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.name or "")
        self.sess = Session(args.out, name)
        meta = {"tool": f"e310lab {VERSION}", "start": iso(now()), "e310": args.e310,
                "gnss": args.gnss, "gnss_baud": args.gnss_baud, "gnss_type": args.gnss_type,
                "kpi_period_s": args.kpi, "putty_port": args.putty_port,
                "platform": sys.platform, "simulate": args.simulate, "name": args.name}
        self.sess.write_meta(meta)
        self.meta = meta
        self.bridge = bool(args.putty_port) and bool(args.e310 or args.simulate)
        world = SimWorld() if args.simulate else None
        p_e310 = p_gnss = None
        if args.simulate:
            p_e310, p_gnss = SimPort("e310", world), SimPort("gnss", world)
        else:
            if args.e310:
                p_e310 = open_serial(args.e310, args.e310_baud)
            if args.gnss:
                p_gnss = open_serial(args.gnss, args.gnss_baud)
        fwd = open_serial(args.gnss_forward, args.gnss_baud) if (args.gnss_forward and not args.simulate) else None
        self.e310 = (E310Console(p_e310, self.sess, show=not self.bridge,
                                 putty_port=args.putty_port if self.bridge else 0)
                     if p_e310 else None)
        self.gnss = (GnssReceiver(p_gnss, self.sess, args.gnss_type, args.gnss_tcp, fwd)
                     if p_gnss else None)
        self.kpi_period = args.kpi if self.e310 else 0
        self.watch = args.watch
        threading.Thread(target=self._background, daemon=True).start()

    def mark(self, label):
        t = now()
        self.sess.row(self.sess.w_mark, [f"{t:.3f}", iso(t), label])
        self.sess.console_line(t, "mark", label)
        print(f"[mark {iso(t)[11:19]}] {label}")

    def status_line(self):
        g = self.gnss.summary() if self.gnss else "GNSS: no receiver port"
        k = self.e310.kpi_last if self.e310 else {}
        e = ""
        if k:
            e = (f"\n          E310: mode {k.get('mode', '?')} {k.get('state', '?')}  J/N {k.get('jn', '-')} dB"
                 f"  supp {k.get('supp', '-')} dB  RX gain {k.get('gain', '-')} dB"
                 f"  floor {k.get('nf', '-')} dBFS  out {k.get('out', '-')} dBFS")
        seg = f"   [segment: {self.seg}]" if self.seg else ""
        return f"[{iso(now())[11:19]}] {g}{e}{seg}"

    def _background(self):
        t_kpi = t_watch = now()
        while not self.quit:
            time.sleep(0.2)
            if self.e310 and self.kpi_period and now() - t_kpi >= self.kpi_period:
                if self.e310.poll_kpi() is not None or now() - t_kpi > 5 * self.kpi_period:
                    t_kpi = now()
            if self.watch and now() - t_watch >= self.watch:
                t_watch = now()
                print(self.status_line())

    def command(self, line):
        line = line.strip()
        if self.waiting and line.lower() in ("", "go", "ok", "y"):
            self.go.set()
            return
        if line.startswith(":"):
            line = line[1:].strip()
        parts = line.split(None, 1)
        if not parts:
            print(self.status_line())
            return
        c, arg = parts[0].lower(), (parts[1].strip() if len(parts) > 1 else "")
        if c in ("help", "h", "?"):
            print(HELP)
        elif c in ("status", "st", "gnss"):
            print(self.status_line())
        elif c == "mark":
            self.mark(arg or "mark")
        elif c == "seg":
            if self.seg:
                self.mark(f"SEG END {self.seg}")
            self.seg = arg or f"segment {iso(now())[11:19]}"
            self.mark(f"SEG START {self.seg}")
        elif c == "end":
            if self.seg:
                self.mark(f"SEG END {self.seg}")
                self.seg = None
        elif c == "sats":
            if self.gnss:
                self.gnss.sat_table()
        elif c == "kpi":
            self.kpi_period = 0 if arg == "off" else float(arg or 2)
            print(f"[e310lab] gnss_kpi? every {self.kpi_period or 'never'} s")
        elif c == "watch":
            self.watch = 0 if arg in ("off", "0") else float(arg or 10)
            print(f"[e310lab] live status every {self.watch or 'never'} s")
        elif c == "restart" and self.gnss:
            self.mark(f"RESTART {arg or 'cold'}")
            self.gnss.restart(arg or "cold")
        elif c == "ttff" and self.gnss:
            self.mark(f"RESTART {arg or 'cold'} (manual)")
            self.gnss.ttff_manual(arg or "cold")
        elif c == "e310" and self.e310:
            threading.Thread(target=self.e310.send, args=(arg,), daemon=True).start()
        elif c == "run":
            threading.Thread(target=PlanRunner(self).run, args=(arg, self.args.yes), daemon=True).start()
        elif c == "plot":
            analyze(self.sess.dir, self.expect)
        elif c in ("quit", "exit", "q", "stop"):
            self.quit = True
        elif not self.bridge and self.e310:
            threading.Thread(target=self.e310.send, args=(line,), daemon=True).start()
        else:
            print(f"unknown command '{c}' (help lists them; E310 commands go in PuTTY)")

    def banner(self):
        a = self.args
        print("\n" + "=" * 72)
        print(f" e310lab {VERSION}   TEST RUNNING   session: {self.sess.dir}")
        print("=" * 72)
        if self.bridge:
            print(f" E310 console : {a.e310 or 'simulated'} -> PuTTY: Session > Host Name 127.0.0.1,")
            print(f"                Port {a.putty_port}, Connection type Raw; Terminal > Local echo")
            print("                'Force off', Local line editing 'Force off'. Then Open.")
        elif self.e310:
            print(f" E310 console : {a.e310 or 'simulated'} (type E310 commands in this window)")
        else:
            print(" E310 console : not logged (PuTTY has the COM port itself)")
        if self.gnss:
            print(f" GNSS receiver: {a.gnss or 'simulated'} @ {a.gnss_baud} ({a.gnss_type})")
            if a.gnss_tcp:
                print(f" GNSS viewer  : TCP 127.0.0.1:{a.gnss_tcp}")
            if a.gnss_forward:
                print(f" GNSS viewer  : open the other end of the virtual pair {a.gnss_forward}")
        else:
            print(" GNSS receiver: none")
        print("-" * 72)
        print(" This window: live status every "
              f"{self.watch or '-'} s.  status  sats  mark <text>  seg <label>  end")
        print("              restart cold   help   QUIT (q / Ctrl+C) = stop + report")
        print("=" * 72 + "\n")

    def loop(self):
        self.banner()
        if self.args.plan:
            threading.Thread(target=self._plan_then_quit, daemon=True).start()
        while not self.quit:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                break
            self.command(line.rstrip("\r\n"))
        self.finish()

    def _plan_then_quit(self):
        time.sleep(1.0)
        PlanRunner(self).run(self.args.plan, self.args.yes)
        if self.args.exit_after_plan:
            self.quit = True
            print("[e310lab] plan done: press Enter to finish")

    def finish(self):
        self.quit = True
        if self.seg:
            self.mark(f"SEG END {self.seg}")
        self.meta["end"] = iso(now())
        self.meta["expect"] = self.expect
        self.sess.write_meta(self.meta)
        if self.e310:
            self.e310.close()
        if self.gnss:
            self.gnss.close()
        time.sleep(0.3)
        self.sess.close()
        print(f"\n[e310lab] TEST STOPPED. Session saved in {self.sess.dir}")
        if not self.args.no_report:
            analyze(self.sess.dir, self.expect)
            print(f"\n[e310lab] open {Path(self.sess.dir) / 'report' / 'report.html'} in a browser;"
                  " zip the session folder to share it.")


# =============================================================================
#  Start-up questions
# =============================================================================
CFG_FILE = Path(__file__).with_name("e310lab_config.json")


def ask(q, default=""):
    d = f" [{default}]" if default != "" else ""
    try:
        a = input(f"  {q}{d}: ").strip()
    except EOFError:
        a = ""
    return a if a else str(default)


def ask_num(q, default, cast=float):
    while True:
        a = ask(q, default)
        try:
            return cast(float(a)) if cast is int else cast(a)
        except ValueError:
            print("    please type a number")


def port_choice(q, ports, default):
    a = ask(q, default)
    if a.isdigit() and 1 <= int(a) <= len(ports):
        return ports[int(a) - 1]
    return "" if a.lower() in ("none", "no", "-", "0") else a


def wizard():
    cfg = {}
    if CFG_FILE.exists():
        try:
            cfg = json.loads(CFG_FILE.read_text())
        except ValueError:
            cfg = {}
    print("\n" + "=" * 72)
    print(f" e310lab {VERSION}  -  E310 anti-jam test logger: SET-UP")
    print(" Press Enter to keep the value in [brackets] (last time's choice).")
    print("=" * 72)
    ports = []
    try:
        from serial.tools import list_ports as lp
        found = list(lp.comports())
        ports = [p.device for p in found]
        print(" Serial ports on this PC:")
        for i, p in enumerate(found, 1):
            print(f"   {i}. {p.device:14s} {p.description}")
        if not found:
            print("   (none found)")
    except ImportError:
        print(" pyserial missing: pip install pyserial")
    print("   type the number or the name; 'sim' = simulated hardware (dry run)\n")
    a = argparse.Namespace()
    a.e310 = port_choice("E310 console port (the one PuTTY used; 'none' = not logged)", ports,
                         cfg.get("e310", "none"))
    a.simulate = a.e310.lower() == "sim"
    if a.simulate:
        a.e310 = ""
    a.e310_baud = ask_num("E310 baud", cfg.get("e310_baud", 115200), int)
    a.putty_port = 0
    if a.e310 or a.simulate:
        a.putty_port = ask_num("TCP port PuTTY connects to (0 = type commands in this window)", cfg.get("putty_port", 5001), int)
    if a.simulate:
        a.gnss = ""
    else:
        a.gnss = port_choice("GNSS receiver port (the one the GNSS viewer used; 'none' = no receiver)",
                             ports, cfg.get("gnss", "none"))
    a.gnss_baud = ask_num("GNSS receiver baud", cfg.get("gnss_baud", 115200), int)
    a.gnss_type = ask("Receiver type: skytraq / ublox / nmea", cfg.get("gnss_type", "skytraq")).lower()
    if a.gnss_type not in ("skytraq", "ublox", "nmea"):
        a.gnss_type = "nmea"
    a.gnss_tcp = ask_num("GNSS viewer over TCP: port (0 = off)", cfg.get("gnss_tcp", 5000), int)
    fw = ask("GNSS viewer over a virtual COM pair: this program's end, e.g. COM11 ('none' = off)",
             cfg.get("gnss_forward", "none"))
    a.gnss_forward = "" if fw.lower() in ("none", "no", "-", "0") else fw
    a.kpi = ask_num("Anti-jam figures (gnss_kpi?) every N s (0 = off)", cfg.get("kpi", 2), float)
    a.watch = ask_num("Live status in this window every N s (0 = off)", cfg.get("watch", 10), float)
    a.name = ask("Test name (folder name), e.g. barrage_0dB", "")
    a.out = ask("Save sessions in folder", cfg.get("out", "sessions"))
    plan = ask("Test plan file (Enter = none, e.g. plans/barrage_56mhz.json)", "")
    a.plan = plan or None
    a.exit_after_plan = False
    a.yes = a.simulate
    a.no_report = False
    keep = {k: getattr(a, k) for k in ("e310", "e310_baud", "putty_port", "gnss", "gnss_baud",
                                       "gnss_type", "gnss_tcp", "gnss_forward", "kpi", "watch", "out")}
    if a.simulate:
        keep["e310"] = "sim"
    keep["e310"] = keep["e310"] or ("sim" if a.simulate else "none")
    keep["gnss"] = keep["gnss"] or "none"
    keep["gnss_forward"] = keep["gnss_forward"] or "none"
    try:
        CFG_FILE.write_text(json.dumps(keep, indent=2))
    except OSError:
        pass
    print("\n  Before pressing Enter: CLOSE PuTTY and the GNSS viewer if they have the COM ports open.")
    input("  Press Enter to START the test ")
    return a


# =============================================================================
#  Analysis
# =============================================================================
def read_csv(p):
    if not p.exists():
        return []
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def segments(markers, t_start, t_end):
    """SEG START/END pairs; otherwise every marker opens a segment to the next."""
    segs, open_ = [], {}
    for m in markers:
        t, lab = float(m["t"]), m["label"]
        if lab.startswith("SEG START "):
            open_[lab[10:]] = t
        elif lab.startswith("SEG END "):
            k = lab[8:]
            if k in open_:
                segs.append((k, open_.pop(k), t))
    for k, t in open_.items():
        segs.append((k, t, t_end))
    if not segs:
        pts = [(float(m["t"]), m["label"]) for m in markers
               if not m["label"].startswith(("PLAN", "OPERATOR"))]
        bounds = [(t_start, "start")] + pts + [(t_end, "end")]
        for (a, la), (b, _) in zip(bounds, bounds[1:]):
            if b - a >= 5:
                segs.append((la, a, b))
    return sorted(segs, key=lambda s: s[1])


def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None
    return sum(v) / len(v)


def analyze(sdir, expect=None):
    sdir = Path(sdir)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed: pip install matplotlib  (logs are kept)")
        plt = None
    meta = json.loads((sdir / "session.json").read_text()) if (sdir / "session.json").exists() else {}
    expect = expect or meta.get("expect", {})
    ep = read_csv(sdir / "gnss_epochs.csv")
    sats = read_csv(sdir / "gnss_sats.csv")
    marks = read_csv(sdir / "markers.csv")
    ttff = read_csv(sdir / "ttff.csv")
    kpi = []
    if (sdir / "kpi.jsonl").exists():
        for l in (sdir / "kpi.jsonl").read_text().splitlines():
            try:
                kpi.append(json.loads(l))
            except ValueError:
                pass
    ts = [float(r["t"]) for r in ep] + [k["t"] for k in kpi] + [float(m["t"]) for m in marks]
    if not ts:
        print("[analyze] nothing recorded yet")
        return
    t0, t1 = min(ts), max(ts)
    rel = lambda t: (float(t) - t0) / 60.0          # minutes
    out = sdir / "report"
    out.mkdir(exist_ok=True)

    # kpi.csv (wide)
    if kpi:
        keys = ["t"] + sorted({k for d in kpi for k in d if k != "t"})
        with open(out / "kpi.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(keys)
            for d in kpi:
                w.writerow([d.get(k, "") for k in keys])

    # ---- segment table ----------------------------------------------------------
    segs = segments(marks, t0, t1)
    rows = []
    for label, a, b in segs:
        e = [r for r in ep if a <= float(r["t"]) <= b]
        k = [d for d in kpi if a <= d["t"] <= b]
        n = len(e)
        fixpct = 100.0 * sum(1 for r in e if int(r["fix_q"] or 0) > 0) / n if n else None
        row = {"segment": label, "start": iso(a)[11:19], "dur_s": round(b - a),
               "epochs": n, "fix_%": fixpct,
               "used": stats([fnum(r["used"]) for r in e]),
               "tracked": stats([fnum(r["tracked"]) for r in e])}
        for c in CONST_NAMES[:4]:
            row[f"trk_{c[:3]}"] = stats([fnum(r[f"trk_{c}"]) for r in e])
            row[f"cn0_{c[:3]}"] = stats([fnum(r[f"cn0_{c}"]) for r in e])
        row["cn0_top4"] = stats([fnum(r["cn0_top4"]) for r in e])
        for kk in ("jn", "supp", "gain", "nf", "out", "wn"):
            row[kk] = stats([fnum(d.get(kk)) for d in k])
        modes = sorted({str(d.get("mode")) for d in k if d.get("mode") is not None})
        row["mode"] = "/".join(modes)
        verdict = ""
        ex = expect.get(label)
        if ex:
            bad = []
            if "fix_pct_min" in ex and (fixpct or 0) < ex["fix_pct_min"]:
                bad.append(f"fix {fixpct or 0:.0f}%<{ex['fix_pct_min']}")
            if "used_min" in ex and (row["used"] or 0) < ex["used_min"]:
                bad.append(f"used<{ex['used_min']}")
            if "cn0_top4_min" in ex and (row["cn0_top4"] or 0) < ex["cn0_top4_min"]:
                bad.append(f"top4<{ex['cn0_top4_min']}")
            if "supp_min" in ex and (row["supp"] or 0) < ex["supp_min"]:
                bad.append(f"supp<{ex['supp_min']}")
            verdict = "PASS" if not bad else "FAIL " + ", ".join(bad)
        row["verdict"] = verdict
        rows.append(row)

    cols = ["segment", "start", "dur_s", "mode", "fix_%", "used", "tracked",
            "trk_GPS", "trk_GLO", "trk_Gal", "trk_Bei",
            "cn0_GPS", "cn0_GLO", "cn0_Gal", "cn0_Bei", "cn0_top4",
            "jn", "supp", "gain", "nf", "out", "verdict"]
    def cell(v):
        if v is None:
            return "-"
        if isinstance(v, float):
            return f"{v:.1f}"
        return str(v)
    with open(out / "segments.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in rows:
            w.writerow([cell(r.get(c)) for c in cols])
    md = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    md += ["| " + " | ".join(cell(r.get(c)) for c in cols) + " |" for r in rows]

    # ---- plots ---------------------------------------------------------------------
    pngs = []
    def vlines(ax):
        for m in marks:
            lab = m["label"]
            if lab.startswith(("SEG END", "PLAN", "OPERATOR")):
                continue
            x = rel(m["t"])
            ax.axvline(x, color="0.5", lw=0.8, ls="--")
            ax.text(x, ax.get_ylim()[1], " " + lab.replace("SEG START ", "")[:28], rotation=90,
                    va="top", ha="left", fontsize=7, color="0.3")

    colors = {"GPS": "#1f77b4", "GLONASS": "#d62728", "Galileo": "#2ca02c", "BeiDou": "#ff7f0e",
              "QZSS": "#9467bd"}
    if plt and ep:
        x = [rel(r["t"]) for r in ep]
        fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        for c in CONST_NAMES[:4]:
            ax[0].plot(x, [fnum(r[f"trk_{c}"]) for r in ep], label=f"{c} tracked", color=colors[c])
        ax[0].plot(x, [fnum(r["used"]) for r in ep], "k", lw=2, label="used in fix")
        ax[0].set_ylabel("satellites")
        ax[0].legend(loc="upper right", fontsize=8, ncol=5)
        ax[0].grid(alpha=0.3)
        vlines(ax[0])
        ax[1].step(x, [int(r["fix_q"] or 0) for r in ep], where="post", color="k")
        ax[1].set_ylabel("GGA fix quality")
        ax[1].set_xlabel("minutes")
        ax[1].grid(alpha=0.3)
        vlines(ax[1])
        fig.suptitle("Satellites per constellation and fix")
        fig.tight_layout()
        fig.savefig(out / "1_satellites.png", dpi=110)
        plt.close(fig)
        pngs.append("1_satellites.png")

        fig, ax = plt.subplots(figsize=(13, 5))
        for c in CONST_NAMES[:4]:
            ax.plot(x, [fnum(r[f"cn0_{c}"]) for r in ep], label=f"{c} mean", color=colors[c])
        ax.plot(x, [fnum(r["cn0_top4"]) for r in ep], "k", lw=2, label="top-4 mean")
        ax.set_ylabel("C/N0 dB-Hz")
        ax.set_xlabel("minutes")
        ax.legend(loc="lower right", fontsize=8, ncol=5)
        ax.grid(alpha=0.3)
        vlines(ax)
        ax.set_title("C/N0 per constellation")
        fig.tight_layout()
        fig.savefig(out / "2_cn0.png", dpi=110)
        plt.close(fig)
        pngs.append("2_cn0.png")

    if plt and sats:
        keys = sorted({(r["system"], int(r["prn"])) for r in sats},
                      key=lambda k: (CONST_NAMES.index(k[0]) if k[0] in CONST_NAMES else 9, k[1]))
        idx = {k: i for i, k in enumerate(keys)}
        tt = sorted({r["t"] for r in sats})
        ti = {t: i for i, t in enumerate(tt)}
        import numpy as np
        img = np.full((len(keys), len(tt)), np.nan)
        for r in sats:
            v = fnum(r["cn0"])
            img[idx[(r["system"], int(r["prn"]))], ti[r["t"]]] = v if v else np.nan
        fig, ax = plt.subplots(figsize=(13, max(4, 0.22 * len(keys) + 1.5)))
        ext = [rel(tt[0]), rel(tt[-1]), len(keys) - 0.5, -0.5]
        im = ax.imshow(img, aspect="auto", interpolation="nearest", extent=ext, cmap="viridis",
                       vmin=20, vmax=50)
        ax.set_yticks(range(len(keys)))
        ax.set_yticklabels([f"{s[:3]} {p}" for s, p in keys], fontsize=7)
        ax.set_xlabel("minutes")
        fig.colorbar(im, ax=ax, label="C/N0 dB-Hz")
        vlines(ax)
        ax.set_title("C/N0 per satellite (blank = not tracked)")
        fig.tight_layout()
        fig.savefig(out / "3_cn0_heatmap.png", dpi=110)
        plt.close(fig)
        pngs.append("3_cn0_heatmap.png")

    if plt and kpi:
        plots = [("jn", "J/N at ADC dB"), ("supp", "suppression dB"), ("gain", "RX gain dB"),
                 ("nf", "noise floor dBFS"), ("out", "output dBFS"), ("wn", "sum|w|^2")]
        plots = [p for p in plots if any(fnum(d.get(p[0])) is not None for d in kpi)]
        if plots:
            fig, ax = plt.subplots(len(plots), 1, figsize=(13, 2.0 * len(plots) + 1), sharex=True)
            ax = ax if len(plots) > 1 else [ax]
            xk = [rel(d["t"]) for d in kpi]
            for a, (k, lab) in zip(ax, plots):
                a.plot(xk, [fnum(d.get(k)) for d in kpi], ".-", ms=3)
                a.set_ylabel(lab, fontsize=8)
                a.grid(alpha=0.3)
                vlines(a)
            ax[-1].set_xlabel("minutes")
            fig.suptitle("E310 anti-jam figures (gnss_kpi?)")
            fig.tight_layout()
            fig.savefig(out / "4_antijam_kpi.png", dpi=110)
            plt.close(fig)
            pngs.append("4_antijam_kpi.png")

    if plt and rows:
        lab = [r["segment"][:22] for r in rows]
        fig, ax = plt.subplots(1, 2, figsize=(13, max(3, 0.4 * len(rows) + 1.5)))
        y = range(len(rows))
        ax[0].barh(y, [r["cn0_top4"] or 0 for r in rows], color="#1f77b4")
        ax[0].set_yticks(list(y))
        ax[0].set_yticklabels(lab, fontsize=8)
        ax[0].invert_yaxis()
        ax[0].set_xlabel("top-4 C/N0 dB-Hz")
        ax[1].barh(y, [r["used"] or 0 for r in rows], color="k")
        ax[1].set_yticks(list(y))
        ax[1].set_yticklabels([])
        ax[1].invert_yaxis()
        ax[1].set_xlabel("satellites used")
        fig.suptitle("Per segment")
        fig.tight_layout()
        fig.savefig(out / "5_segments.png", dpi=110)
        plt.close(fig)
        pngs.append("5_segments.png")

    # ---- report ------------------------------------------------------------------------
    tt_md = ["| time | mode | TTFF s | result |", "|---|---|---|---|"]
    tt_md += [f"| {r['time'][11:19]} | {r['mode']} | {r['ttff_s'] or '-'} | {r['result']} |" for r in ttff]
    summary = ["# E310 anti-jam test report", "",
               f"- session: `{sdir.name}`  ({meta.get('name') or ''})",
               f"- start {iso(t0)}, length {(t1 - t0) / 60:.1f} min, "
               f"{len(ep)} GNSS epochs, {len(kpi)} KPI readings, {len(marks)} markers",
               f"- E310 {meta.get('e310')}, receiver {meta.get('gnss')} ({meta.get('gnss_type')})", "",
               "## Segments", ""] + md + ["", "## TTFF", ""] + (tt_md if ttff else ["(none)"]) + \
              ["", "## Markers", ""] + [f"- {m['time'][11:19]}  {m['label']}" for m in marks] + \
              ["", "## Plots", ""] + [f"![{p}]({p})" for p in pngs]
    (out / "summary.md").write_text("\n".join(summary), encoding="utf-8")

    def htable(header, body):
        h = "<tr>" + "".join(f"<th>{html.escape(c)}</th>" for c in header) + "</tr>"
        b = "".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in r) + "</tr>"
                    for r in body)
        return f"<table>{h}{b}</table>"
    page = f"""<!DOCTYPE html><html><head><meta charset="utf-8"><title>E310 test {html.escape(sdir.name)}</title>
<style>body{{font-family:Segoe UI,Arial,sans-serif;margin:24px;color:#222}}
table{{border-collapse:collapse;font-size:12px;margin:8px 0}}td,th{{border:1px solid #ccc;padding:3px 6px;text-align:right}}
th{{background:#f0f0f0}}td:first-child{{text-align:left}}img{{max-width:100%;border:1px solid #ddd;margin:8px 0}}
.fail{{color:#b00}}</style></head><body>
<h1>E310 anti-jam test report</h1>
<p>Session <b>{html.escape(sdir.name)}</b> {html.escape(meta.get('name') or '')}<br>
start {iso(t0)}, {(t1 - t0) / 60:.1f} min, {len(ep)} GNSS epochs, {len(kpi)} KPI readings</p>
<h2>Segments</h2>{htable(cols, [[cell(r.get(c)) for c in cols] for r in rows])}
<h2>TTFF</h2>{htable(['time', 'mode', 'TTFF s', 'result'], [[r['time'][11:19], r['mode'], r['ttff_s'] or '-', r['result']] for r in ttff]) if ttff else '<p>(none)</p>'}
<h2>Plots</h2>{''.join(f'<img src="{p}">' for p in pngs)}
<h2>Markers</h2>{htable(['time', 'label'], [[m['time'][11:19], m['label']] for m in marks])}
</body></html>"""
    (out / "report.html").write_text(page, encoding="utf-8")
    print(f"[analyze] report: {out / 'report.html'}")
    print("\n".join(md))


# =============================================================================
#  main
# =============================================================================
def main():
    if os.name == "nt":
        os.system("")          # enable ANSI colours in the Windows console
    if len(sys.argv) == 1:     # double-click / no arguments: ask everything
        args = wizard()
        Lab(args).loop()
        return
    ap = argparse.ArgumentParser(description="ANTSDR E310 anti-jam test logger "
                                 "(E310 console + GNSS receiver). No arguments = set-up questions.")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("ports", help="list serial ports")
    r = sub.add_parser("run", help="start a test with the options on the command line")
    r.add_argument("--e310", default="", help="E310 console port, e.g. COM5 or /dev/ttyUSB0")
    r.add_argument("--e310-baud", type=int, default=115200)
    r.add_argument("--putty-port", type=int, default=5001,
                   help="TCP port PuTTY connects to (Raw); 0 = type E310 commands in this window")
    r.add_argument("--gnss", default="", help="GNSS receiver port, e.g. COM7 or /dev/ttyACM0")
    r.add_argument("--gnss-baud", type=int, default=115200)
    r.add_argument("--gnss-type", choices=["nmea", "skytraq", "ublox"], default="skytraq")
    r.add_argument("--gnss-tcp", type=int, default=0, help="re-serve the receiver stream on TCP")
    r.add_argument("--gnss-forward", default="", help="re-serve it on a virtual serial port")
    r.add_argument("--kpi", type=float, default=2.0, help="gnss_kpi? period in s (0 = off)")
    r.add_argument("--watch", type=float, default=10.0, help="live status period in s (0 = off)")
    r.add_argument("--name", default="")
    r.add_argument("--out", default="sessions")
    r.add_argument("--plan")
    r.add_argument("--exit-after-plan", action="store_true")
    r.add_argument("--yes", action="store_true", help="plan prompts continue by themselves")
    r.add_argument("--no-report", action="store_true")
    r.add_argument("--simulate", action="store_true", help="dry run without hardware")
    an = sub.add_parser("analyze", help="make the plots and tables of a session")
    an.add_argument("session")
    args = ap.parse_args()
    if args.cmd == "ports":
        list_ports()
    elif args.cmd == "run":
        if not (args.simulate or args.e310 or args.gnss):
            ap.error("give --e310 and/or --gnss (or --simulate); 'ports' lists them")
        Lab(args).loop()
    elif args.cmd == "analyze":
        analyze(args.session)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
