#!/usr/bin/env python3
"""
e310_autotest.py  --  automatic anti-jam test runner for the E310 SMI-STAP
(CW and wideband), built on e310_gnss_logger.py (keep both files in one folder).

It drives the E310 console (as PuTTY would), COLD-STARTS the SkyTraq receiver
itself, times TTFF from the receiver's own NMEA, and measures the C/N0 of the
best 6 satellites 60 s after each fix. You only change the jammer when asked.

    python e310_autotest.py --e310 COM12 --gnss COM4                 (full plan)
    python e310_autotest.py --e310 COM12 --gnss COM4 --plan wb --runs 2
    python e310_autotest.py --e310 COM12 --gnss COM4 --share COM20   (GNSS Viewer on COM21)

Close PuTTY and GNSS Viewer first (or use --share for the viewer).

PLANS (--plan)
    full      baseline -> CW sweep -> wideband: best band -> best clip limit -> power sweep
    baseline  no jammer, bands 3 / 4 / 6
    cw        CW sweep only (band 3)
    wb        wideband only: best band -> best clip limit -> power sweep
    The wideband part CHOOSES as it goes: the band with the best result at
    10 dB is used for the clip-limit test, and the best of both for the sweep.

AT EACH JAMMER CHANGE it stops and asks:   set the jammer, then
    Enter = go     s = skip this step     q = stop and write the results

OUTPUT (folder logs/YYYYMMDD_HHMMSS_autotest[_name]/)
    results.csv     one row per cold start: TTFF, best-6 C/N0, satellites used,
                    HDOP + the E310 figures (gain, J/N, suppression, J/N out,
                    noise floor, clip rate, hunts, DAC clips during the run)
    summary.csv     one row per step: the means of its runs + a score
    report.txt      the ranking and the settings it recommends
    + everything the logger writes (console log, NMEA log, gnss_1s.csv, chart.png)
Zip the folder and send it.

Needs: pip install pyserial   (matplotlib optional, for chart.png)
"""
import argparse
import csv
import os
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import e310_gnss_logger as lg
except ImportError:
    sys.exit("e310_gnss_logger.py must be in the same folder as this script")

KPI_KEYS = ["gain", "gain2", "jn", "supp", "jnout", "cn0loss", "nf", "floor", "pk1", "pk2",
            "cliprate", "hunts", "dacclip", "steady", "keff", "state", "agcrule", "taps"]
NOISE_RE = re.compile(r"NOISE\s+floor\s+(-?[\d.]+)\s+dBFS\s+AD9361 internal\s+(-?[\d.]+)")
DACCLIP_RE = re.compile(r"DAC clips\s+(\d+)")
GAIN_RE = re.compile(r"RX\s+gain RX1\s+(\d+)\s+dB")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def fmt(v, nd=1):
    return "" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


class Stop(Exception):
    pass


class Skip(Exception):
    pass


class AutoLogger(lg.Logger):
    """The logger with a console line buffer, an echo switch and a quieter screen."""

    def __init__(self, a):
        super().__init__(a)
        self.lines = []            # (t, line)
        self.lines_lock = threading.Lock()
        self.echo = a.echo
        self.ack_t = 0.0

    def e310_reader(self):
        buf = b""
        while not self.stop.is_set():
            try:
                data = self.e310.read(512)
            except serial.SerialException as e:
                print(f"\n[autotest] E310 port error: {e}")
                self.stop.set()
                break
            if not data:
                continue
            if self.echo:
                sys.stdout.write(data.decode("ascii", "replace"))
                sys.stdout.flush()
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.e310_line(lg.ANSI.sub("", raw.rstrip(b"\r").decode("ascii", "replace")))

    def e310_line(self, line, t=None):
        t = t or lg.now()
        super().e310_line(line, t)
        with self.lines_lock:
            self.lines.append((t, line))
            if len(self.lines) > 20000:
                del self.lines[:5000]
        if not self.echo and re.search(r"JAMMER (ON|OFF)|NULLED|AUTO-START:|RESULT:", line):
            print(f"    [E310] {line.strip()}")

    def gnss_reader(self):
        """As the logger's, but remembers when the restart ACK came."""
        while not self.stop.is_set():
            try:
                raw = self.gnss.readline()
                if raw and self.share:
                    self.share.write(raw)
                if b"\xa0\xa1\x00\x02\x83\x01" in raw:
                    self.ack_t = lg.now()
                    self.marker("gnss", "restart ACK")
                elif b"\xa0\xa1\x00\x02\x84\x01" in raw:
                    self.marker("gnss", "restart NACK")
                ln = raw.decode("ascii", "replace").strip()
            except serial.SerialException as e:
                print(f"\n[autotest] GNSS port error: {e}")
                self.stop.set()
                break
            if not ln or "$" not in ln:
                continue
            ln = ln[ln.index("$"):]
            t = lg.now()
            with self.flock:
                self.nmea_log.write(f"[{lg.tstr(t)}] {ln}\n")
            self.state.feed(ln, t)

    def lines_since(self, t0):
        with self.lines_lock:
            return [ln for t, ln in self.lines if t >= t0]

    # ---------------------------------------------------------- receiver figures
    def sats_now(self):
        """(fix, used, hdop, sorted C/N0 of every tracked satellite)."""
        t = lg.now()
        st = self.state
        with st.lock:
            cn = sorted([c for (c, ts) in st.sv.values() if c and t - ts < lg.STALE_S], reverse=True)
            fresh = t - st.t_gga < lg.STALE_S
            return (st.fix if fresh else 0), (st.nused if fresh else 0), st.hdop, cn


class Runner:
    def __init__(self, L, a):
        self.L = L
        self.a = a
        self.cur = {"band": None, "clip": None}
        self.jammer = None
        self.res_f = open(os.path.join(L.dir, "results.csv"), "w", newline="", encoding="utf-8")
        self.res = csv.writer(self.res_f)
        self.res.writerow(["step", "phase", "jammer", "band", "agcclip", "run", "ttff_s", "fix",
                           "cn0_best6", "cn0_best4", "cn0_used_mean", "sats_tracked", "sats_used", "hdop",
                           "fix_lost_s", "noise_floor_dbfs", "ad9361_floor_dbfs"]
                          + [f"kpi_{k}" for k in KPI_KEYS] + ["dacclip_during_run"])
        self.steps = []          # summaries

    # ------------------------------------------------------------ console
    def cmd(self, c, wait=1.0):
        print(f"    > {c}")
        self.L.send(c)
        self.sleep(wait)

    def sleep(self, s):
        end = lg.now() + s
        while lg.now() < end:
            if self.L.stop.is_set():
                raise Stop()
            time.sleep(min(0.2, max(0.0, end - lg.now())))

    def kpi(self):
        t0 = lg.now()
        self.L.send("gnss_kpi?")
        for _ in range(30):
            self.sleep(0.1)
            got = [d for d in self.L.kpi if fnum(d.get("t_s")) is not None
                   and fnum(d["t_s"]) + self.L.t0 >= t0 - 0.05]
            if got:
                return got[-1]
        return {}

    def status(self):
        t0 = lg.now()
        self.L.send("status?")
        self.sleep(1.5)
        nf = adf = dac = gain = None
        for ln in self.L.lines_since(t0):
            m = NOISE_RE.search(ln)
            if m:
                nf, adf = float(m.group(1)), float(m.group(2))
            m = DACCLIP_RE.search(ln)
            if m:
                dac = int(m.group(1))
            m = GAIN_RE.search(ln)
            if m:
                gain = int(m.group(1))
        return {"nf": nf, "adf": adf, "dac": dac, "gain": gain}

    # ------------------------------------------------------------ set-up
    def ask_jammer(self, text):
        if text == self.jammer:
            return
        print("\n" + "=" * 78)
        print(f"  SET THE JAMMER:  {text}")
        print("  then press  Enter = go,   s = skip this step,   q = stop and write the results")
        print("=" * 78)
        ans = input("  > ").strip().lower()
        if ans.startswith("q"):
            raise Stop()
        if ans.startswith("s"):
            raise Skip()
        self.jammer = text
        self.L.marker("note", f"JAMMER: {text}")

    def configure(self, band, clip):
        if band is not None and band != self.cur["band"]:
            self.cmd(f"band={band}", wait=self.a.band_wait)
            self.cur["band"] = band
            self.cmd("stap=1", wait=6)
        elif self.cur["band"] is None:
            self.cmd("stap=1", wait=6)
        if clip is not None and clip != self.cur["clip"]:
            self.cmd(f"gnss_agcclip={clip}", wait=1)
            self.cur["clip"] = clip

    # ------------------------------------------------------------ measurement
    def cold_run(self):
        L, a = self.L, self.a
        L.ack_t = 0.0
        L.gnss_cold()
        tc = lg.now()
        # the receiver must first lose the fix (cold start taken), then get one
        lost = False
        t_fix = None
        last_print = 0
        while lg.now() - tc < a.fix_timeout:
            self.sleep(0.2)
            fix, nused, _, cn = L.sats_now()
            el = lg.now() - tc
            if not lost:
                if fix == 0 or el > 8:
                    lost = True
                continue
            if fix > 0:
                t_fix = lg.now()
                break
            if el - last_print >= 15:
                last_print = el
                print(f"      {el:4.0f} s  no fix yet, tracking {len(cn)} sats, "
                      f"best-4 C/N0 {fmt(mean(cn[:4]))}")
        ttff = (t_fix - tc) if t_fix else None
        print(f"      TTFF {fmt(ttff, 0) + ' s' if ttff else 'NO FIX in ' + str(int(a.fix_timeout)) + ' s'}"
              f"{'' if L.ack_t else '  (no restart ACK seen: check the receiver port / baud)'}")
        # settle, then average over the window
        if t_fix:
            self.sleep(a.post)
        b6, b4, nu, nt, hd, cu = [], [], [], [], [], []
        lost_s = 0.0
        t_end = lg.now() + a.window
        while lg.now() < t_end:
            self.sleep(1.0)
            fix, nused, hdop, cn = L.sats_now()
            if cn:
                b6.append(mean(cn[:6]))
                b4.append(mean(cn[:4]))
            nt.append(len(cn))
            nu.append(nused)
            if hdop is not None and fix > 0:
                hd.append(hdop)
            if fix == 0:
                lost_s += 1.0
            snap = L.state.snapshot(lg.now())
            cu.append(mean([snap["consts"][c]["cn0_used"] for c in lg.CONSTS]))
        return {"ttff": ttff, "fix": 1 if t_fix else 0, "b6": mean(b6), "b4": mean(b4), "cu": mean(cu),
                "nt": mean(nt), "nu": mean(nu), "hdop": mean(hd), "lost": lost_s}

    def step(self, name, phase, jammer, band, clip, runs=None):
        runs = runs or self.a.runs
        print(f"\n--- STEP {len(self.steps) + 1}: {name}   (band {band}, agcclip {clip}, {runs} cold starts)")
        try:
            self.ask_jammer(jammer)
        except Skip:
            print("    skipped")
            return None
        self.L.marker("note", f"STEP {name}")
        self.configure(band, clip)
        print(f"    settling {self.a.settle:.0f} s with the jammer ...")
        self.sleep(self.a.settle)
        st = self.status()
        k0 = self.kpi()
        print(f"    E310: gain {k0.get('gain', st['gain'])} dB  J/N {k0.get('jn', '-')}  "
              f"supp {k0.get('supp', '-')}  J/N out {k0.get('jnout', '-')}  "
              f"floor {fmt(st['nf'])} dBFS (AD9361 {fmt(st['adf'])})  clip {k0.get('cliprate', '-')} ppm")
        rows = []
        for r in range(1, runs + 1):
            print(f"    cold start {r}/{runs}")
            d_before = fnum(self.kpi().get("dacclip"))
            m = self.cold_run()
            k = self.kpi()
            st2 = self.status()
            d_after = fnum(k.get("dacclip"))
            dd = (d_after - d_before) if (d_after is not None and d_before is not None) else None
            print(f"      best-6 C/N0 {fmt(m['b6'])} dB-Hz, used {fmt(m['nu'])}, HDOP {fmt(m['hdop'])}, "
                  f"fix lost {m['lost']:.0f} s, DAC clips +{fmt(dd, 0) or '?'}")
            row = [len(self.steps) + 1, phase, jammer, band, clip, r, fmt(m["ttff"], 1), m["fix"],
                   fmt(m["b6"]), fmt(m["b4"]), fmt(m["cu"]), fmt(m["nt"]), fmt(m["nu"]), fmt(m["hdop"], 2),
                   f"{m['lost']:.0f}", fmt(st2["nf"] if st2["nf"] is not None else st["nf"]),
                   fmt(st2["adf"] if st2["adf"] is not None else st["adf"])]
            row += [k.get(x, "") for x in KPI_KEYS] + [fmt(dd, 0)]
            self.res.writerow(row)
            self.res_f.flush()
            rows.append({"m": m, "k": k, "dd": dd})
        s = {
            "step": len(self.steps) + 1, "name": name, "phase": phase, "jammer": jammer, "band": band,
            "clip": clip, "runs": len(rows),
            "fix_rate": mean([r["m"]["fix"] for r in rows]),
            "ttff": mean([r["m"]["ttff"] for r in rows]),
            "cn0_best6": mean([r["m"]["b6"] for r in rows]),
            "sats_used": mean([r["m"]["nu"] for r in rows]),
            "hdop": mean([r["m"]["hdop"] for r in rows]),
            "fix_lost_s": mean([r["m"]["lost"] for r in rows]),
            "gain": mean([fnum(r["k"].get("gain")) for r in rows]),
            "jn": mean([fnum(r["k"].get("jn")) for r in rows]),
            "supp": mean([fnum(r["k"].get("supp")) for r in rows]),
            "jnout": mean([fnum(r["k"].get("jnout")) for r in rows]),
            "cliprate": mean([fnum(r["k"].get("cliprate")) for r in rows]),
            "dacclip_per_run": mean([r["dd"] for r in rows]),
            "noise_floor": st["nf"], "ad9361_floor": st["adf"],
        }
        s["score"] = score(s)
        self.steps.append(s)
        print(f"    => fix {fmt(100 * (s['fix_rate'] or 0), 0)} %, TTFF {fmt(s['ttff'], 0)} s, "
              f"best-6 C/N0 {fmt(s['cn0_best6'])} dB-Hz, score {fmt(s['score'])}")
        return s

    # ------------------------------------------------------------ plans
    def baseline(self):
        for b in self.a.bands:
            self.step(f"no jammer, band {b}", "baseline", "OFF (no jammer)", b, self.a.clip0)

    def cw(self):
        for lv in self.a.cw_levels:
            self.step(f"CW {lv} dB", "cw", f"CW at 1575.42 MHz, {lv} dB power", 3, self.a.clip0)
        if self.a.cw_levels:
            top = max(self.a.cw_levels)
            self.step(f"CW {top} dB at the LO", "cw", f"CW at 1582.50 MHz (the LO), {top} dB power", 3, self.a.clip0)

    def wb(self):
        lv0 = self.a.wb_select_level
        jam = f"WIDEBAND noise barrage 56 MHz, {lv0} dB power"
        cand = [s for s in (self.step(f"WB {lv0} dB, band {b}", "wb_band", jam, b, self.a.clip0)
                            for b in self.a.bands) if s]
        best_band = best(cand)["band"] if cand else 3
        print(f"\n    BEST BAND for wideband: {best_band}")
        self.L.marker("note", f"best wideband band {best_band}")
        cand2 = [s for s in cand if s["band"] == best_band]
        for c in self.a.clips:
            if c == self.a.clip0:
                continue
            s = self.step(f"WB {lv0} dB, band {best_band}, agcclip {c}", "wb_clip", jam, best_band, c)
            if s:
                cand2.append(s)
        best_clip = best(cand2)["clip"] if cand2 else self.a.clip0
        print(f"\n    BEST CLIP LIMIT: {best_clip} ppm (band {best_band})")
        self.L.marker("note", f"best agcclip {best_clip}")
        for lv in self.a.wb_levels:
            if lv == lv0:
                continue
            self.step(f"WB {lv} dB (band {best_band}, agcclip {best_clip})", "wb_sweep",
                      f"WIDEBAND noise barrage 56 MHz, {lv} dB power", best_band, best_clip)
        for kind in self.a.extra:
            for lv in (10, 20):
                self.step(f"{kind} {lv} dB (band {best_band}, agcclip {best_clip})", "extra",
                          f"{kind}, {lv} dB power", best_band, best_clip)
        return best_band, best_clip

    # ------------------------------------------------------------ results
    def write(self, best_band=None, best_clip=None):
        self.res_f.close()
        if not self.steps:
            return
        keys = list(self.steps[0].keys())
        with open(os.path.join(self.L.dir, "summary.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for s in self.steps:
                w.writerow({k: (fmt(v) if isinstance(v, float) else v) for k, v in s.items()})
        lines = ["E310 SMI-STAP automatic test", "=" * 100,
                 f"{'step':<44s}{'fix':>6s}{'TTFF':>7s}{'C/N0 b6':>9s}{'used':>6s}{'HDOP':>6s}"
                 f"{'gain':>6s}{'J/N':>6s}{'supp':>6s}{'out':>6s}{'floor':>7s}{'DACclip':>9s}"]
        for s in self.steps:
            lines.append(f"{s['name'][:43]:<44s}{fmt(100 * (s['fix_rate'] or 0), 0) + '%':>6s}"
                         f"{fmt(s['ttff'], 0):>7s}{fmt(s['cn0_best6']):>9s}{fmt(s['sats_used']):>6s}"
                         f"{fmt(s['hdop'], 2):>6s}{fmt(s['gain'], 0):>6s}{fmt(s['jn']):>6s}"
                         f"{fmt(s['supp']):>6s}{fmt(s['jnout']):>6s}{fmt(s['noise_floor']):>7s}"
                         f"{fmt(s['dacclip_per_run'], 0):>9s}")
        lines.append("")
        base = {s["band"]: s for s in self.steps if s["phase"] == "baseline"}
        if base:
            lines.append("C/N0 loss vs the no-jammer baseline on the same band (best-6 C/N0):")
            for s in self.steps:
                b = base.get(s["band"])
                if s["phase"] != "baseline" and b and b["cn0_best6"] and s["cn0_best6"]:
                    lines.append(f"   {s['name'][:50]:<52s}{b['cn0_best6'] - s['cn0_best6']:6.1f} dB")
            lines.append("")
        if best_band is not None:
            lines.append(f"RECOMMENDED for wideband jammers:  band={best_band}   gnss_agcclip={best_clip}")
        lines.append("score = 100 x fix rate + best-6 C/N0 - TTFF/30  (higher is better)")
        txt = "\n".join(lines)
        with open(os.path.join(self.L.dir, "report.txt"), "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")
        print("\n" + txt)


def score(s):
    fr = s["fix_rate"] or 0.0
    cn = s["cn0_best6"] or 0.0
    tt = s["ttff"] if s["ttff"] is not None else 600.0
    return 100.0 * fr + cn - tt / 30.0


def best(steps):
    return max(steps, key=lambda s: s["score"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--e310", help="E310 console port (PuTTY's), e.g. COM12")
    ap.add_argument("--e310-baud", type=int, default=115200)
    ap.add_argument("--gnss", help="SkyTraq receiver port (GNSS Viewer's), e.g. COM4")
    ap.add_argument("--gnss-baud", type=int, default=115200)
    ap.add_argument("--share", default="", help="copy the receiver to this port for GNSS Viewer (com0com end)")
    ap.add_argument("--plan", default="full", choices=["full", "baseline", "cw", "wb"])
    ap.add_argument("--runs", type=int, default=3, help="cold starts per step (default 3)")
    ap.add_argument("--settle", type=float, default=60, help="s with the jammer before measuring (default 60)")
    ap.add_argument("--fix-timeout", type=float, default=300, help="s to wait for a fix (default 300)")
    ap.add_argument("--post", type=float, default=60, help="s after the fix before C/N0 is read (default 60)")
    ap.add_argument("--window", type=float, default=20, help="s of C/N0 averaging (default 20)")
    ap.add_argument("--band-wait", type=float, default=20, help="s after band=N for the restart (default 20)")
    ap.add_argument("--bands", default="3,4,6", help="bands to compare (default 3,4,6)")
    ap.add_argument("--cw-levels", default="0,10,20,30,40")
    ap.add_argument("--wb-levels", default="0,5,10,15,20")
    ap.add_argument("--wb-select-level", type=int, default=10, help="WB power for the band / clip choice")
    ap.add_argument("--clips", default="100,30,0", help="gnss_agcclip values to compare (first = default)")
    ap.add_argument("--extra", default="", help="also test, e.g. 'pulse barrage 20%% duty,chirp'")
    ap.add_argument("--tx1", type=int, default=0, help="send tx1_att= this first (mdB, 0 = leave)")
    ap.add_argument("--echo", action="store_true", help="show the whole E310 console on screen")
    ap.add_argument("--out", default="logs")
    ap.add_argument("--name", default="")
    a = ap.parse_args()
    a.bands = [int(x) for x in a.bands.split(",") if x.strip()]
    a.cw_levels = [int(x) for x in a.cw_levels.split(",") if x.strip()]
    a.wb_levels = [int(x) for x in a.wb_levels.split(",") if x.strip()]
    a.clips = [int(x) for x in a.clips.split(",") if x.strip()]
    a.clip0 = a.clips[0]
    a.extra = [x.strip() for x in a.extra.split(",") if x.strip()]
    a.kpi, a.watch = 0, 0
    a.name = "autotest" + ("_" + a.name if a.name else "")
    if not a.e310 or not a.gnss:
        lg.ask_ports(a)
    if os.name == "nt":
        os.system("")

    L = AutoLogger(a)
    try:
        L.e310 = serial.Serial(a.e310, a.e310_baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {a.e310}: {e}  (close PuTTY first)")
    try:
        L.gnss = L.open_gnss()
    except serial.SerialException as e:
        sys.exit(f"cannot open {a.gnss}: {e}  (close GNSS Viewer first, or use --share)")
    if a.share:
        L.share = lg.SharePort(a.share, L.gnss.baudrate)
        print(f"[autotest] receiver copied to {L.share.name}: open the other end in GNSS Viewer")
    threads = [threading.Thread(target=f, daemon=True) for f in (L.e310_reader, L.gnss_reader, L.gnss_tick)]
    if L.share:
        threads.append(threading.Thread(target=L.share_reader, daemon=True))
    for th in threads:
        th.start()
    print(f"[autotest] logging to {L.dir}")
    L.marker("note", f"autotest plan {a.plan}, {a.runs} cold starts per step")

    R = Runner(L, a)
    bb = bc = None
    t_start = time.time()
    try:
        L.e310.write(b"\r\n")
        R.sleep(1)
        if a.tx1:
            R.cmd(f"tx1_att={a.tx1}")
        R.cmd("gnss_rxoff=0")
        R.cmd("gnss_ajbw=0")
        R.cmd(f"gnss_agcclip={a.clip0}")
        R.cur["clip"] = a.clip0
        if a.plan in ("full", "baseline"):
            R.baseline()
        if a.plan in ("full", "cw"):
            R.cw()
        if a.plan in ("full", "wb"):
            bb, bc = R.wb()
        if bb is not None:
            print(f"\n[autotest] leaving the E310 on the best wideband setting: band={bb} gnss_agcclip={bc}")
            R.configure(bb, bc)
    except (Stop, KeyboardInterrupt, EOFError):
        print("\n[autotest] stopped")
    except Exception as e:                       # never lose the results on an error
        print(f"\n[autotest] error: {e!r} -- writing what was measured")
    print(f"[autotest] ran {(time.time() - t_start) / 60:.0f} min")
    R.write(bb, bc)
    L.finish()


if __name__ == "__main__":
    main()
