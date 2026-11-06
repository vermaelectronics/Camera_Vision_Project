#!/usr/bin/env python3
"""
STAP-PI anti-jam demo: 2-element CRPA vs. a high-power wideband barrage jammer.

Runs the reference scenario, compares spatial-only power inversion (M = 1)
with space-time power inversion (STAP-PI, M taps per element), prints the
anti-jam figures of merit and writes the figures used in the README.

    python3 run_demo.py                     # default: J/S = 95 dB, M = 7, 14-bit ADC
    python3 run_demo.py --js-db 110 --taps 9 --adc-bits 0   # ideal ADC
    python3 run_demo.py --no-plots          # numbers only
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from stap_pi import (  # noqa: E402
    CrpaScenario,
    StapPIConfig,
    StapPIProcessor,
    cone_angle_unit_vector,
    correlate_ca,
    evaluate,
    null_depth_vs_frequency,
    psd,
    snr_gain_vs_frequency,
    space_time_response,
)
from stap_pi.metrics import db  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]

# Reference light-mode chart palette (fixed categorical order, recessive chrome)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS, SURFACE = "#e1e0d9", "#c3c2b7", "#fcfcfb"
BLUES = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


def setup_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "axes.titlesize": 11, "axes.labelsize": 9.5, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
        "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
        "xtick.labelsize": 8.5, "ytick.labelsize": 8.5,
        "legend.frameon": False, "legend.fontsize": 8.5, "legend.labelcolor": INK2,
        "lines.linewidth": 2.0, "lines.solid_capstyle": "round",
        "font.family": "DejaVu Sans", "text.color": INK,
    })
    return plt


def cone_angle(u):
    return float(np.degrees(np.arcsin(np.clip(u[0], -1, 1))))


def run_case(scenario: CrpaScenario, n_taps: int, **cfg):
    data = scenario.generate()
    res = StapPIProcessor(StapPIConfig(n_taps=n_taps, **cfg)).process(data.x)
    return data, res, evaluate(data, res)


def fmt_row(name, m):
    return (f"{name:<22s} {m['suppression_db']:8.1f} {m['jnr_in_db']:8.1f} {m['jnr_out_db']:8.1f} "
            f"{m['cn0_in_db']:8.1f} {m['cn0_out_db']:8.1f} {m['cn0_degradation_db']:8.1f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--js-db", type=float, default=95.0, help="jammer-to-signal ratio at the antenna [dB]")
    ap.add_argument("--taps", type=int, default=7, help="STAP taps per element (M)")
    ap.add_argument("--adc-bits", type=int, default=14, help="ADC resolution, 0 = ideal")
    ap.add_argument("--no-mismatch", action="store_true", help="identical (ideal) RF channels")
    ap.add_argument("--duration", type=float, default=10e-3, help="record length [s]")
    ap.add_argument("--block-len", type=int, default=4096, help="samples per weight update")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "docs" / "figures")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--quick", action="store_true", help="coarser sweeps")
    args = ap.parse_args(argv)

    adc = None if args.adc_bits == 0 else args.adc_bits
    sc = CrpaScenario(js_db=args.js_db, adc_bits=adc, mismatch=not args.no_mismatch, duration_s=args.duration)
    t0 = time.time()
    data = sc.generate()
    taps_set = sorted({1, 3, 5, args.taps, 9} if not args.quick else {1, args.taps})
    results = {}
    for M in taps_set:
        res = StapPIProcessor(StapPIConfig(n_taps=M, block_len=args.block_len)).process(data.x)
        results[M] = (res, evaluate(data, res))

    th_s, th_j = cone_angle(sc.sat_u), cone_angle(sc.jam_u)
    print("STAP-PI 2-element CRPA - wideband barrage jammer")
    print(f"  fc = {sc.fc/1e6:.2f} MHz, fs = {sc.fs/1e6:.1f} MHz, jammer BW = {sc.jam_bw_hz/1e6:.0f} MHz, "
          f"front-end BW = {sc.frontend_bw_hz/1e6:.0f} MHz, ADC = {adc or 'ideal'} bit, mismatch = {sc.mismatch}")
    print(f"  satellite PRN {sc.prn} at {th_s:+.1f} deg off broadside, C/N0 = {sc.cn0_dbhz:.0f} dB-Hz; "
          f"jammer at {th_j:+.1f} deg, J/S = {sc.js_db:.0f} dB")
    print(f"  block = {args.block_len} samples ({args.block_len/sc.fs*1e6:.0f} us), reference = element 0 centre tap\n")
    print(f"{'processor':<22s} {'supp':>8s} {'JNR in':>8s} {'JNR out':>8s} {'C/N0 in':>8s} {'C/N0 out':>8s} {'C/N0 loss':>8s}")
    print(f"{'':<22s} {'[dB]':>8s} {'[dB]':>8s} {'[dB]':>8s} {'[dB-Hz]':>8s} {'[dB-Hz]':>8s} {'[dB]':>8s}")
    for M, (_, m) in results.items():
        name = "spatial PI (M=1)" if M == 1 else f"STAP-PI (M={M})"
        print(fmt_row(name, m))
    m0 = results[taps_set[0]][1]
    print(f"\n  clean single-element reference C/N0 (no jammer): {m0['cn0_ref_db']:.1f} dB-Hz")

    summary = {
        "scenario": {"js_db": sc.js_db, "jnr_in_db": m0["jnr_in_db"], "adc_bits": adc, "mismatch": sc.mismatch,
                     "fs_hz": sc.fs, "jam_bw_hz": sc.jam_bw_hz, "sat_cone_deg": th_s, "jam_cone_deg": th_j,
                     "cn0_ref_dbhz": m0["cn0_ref_db"]},
        "results": {str(M): {k: v for k, v in m.items() if k != "y_parts"} for M, (_, m) in results.items()},
    }

    if args.no_plots:
        print(f"\n(done in {time.time()-t0:.1f} s)")
        return summary

    plt = setup_matplotlib()
    args.out.mkdir(parents=True, exist_ok=True)
    M_main = args.taps
    res_main, m_main = results[M_main]
    res_sp, m_sp = results[1]
    ref = res_main.config.ref_element
    k0 = (res_main.config.weight_latency + 1) * res_main.config.block_len
    freqs = np.linspace(-sc.fs / 2, sc.fs / 2, 601)

    # -- Fig 1: PSD ----------------------------------------------------------
    f, p_noise_ref = psd(data.parts["noise"][ref, k0:], sc.fs, 1024)
    norm = np.median(p_noise_ref[np.abs(f) < 5e6])
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    curves = [
        ("Reference element input (jammed)", data.x[ref, k0:], SERIES[0]),
        ("Spatial PI output (M=1)", res_sp.y[k0:], SERIES[1]),
        (f"STAP-PI output (M={M_main})", res_main.y[k0:], SERIES[2]),
    ]
    for lbl, sig, col in curves:
        ff, pp = psd(sig, sc.fs, 1024)
        ax.plot(ff / 1e6, db(pp / norm), color=col, label=lbl)
    ax.plot(f / 1e6, db(p_noise_ref / norm), color=MUTED, lw=1.2, label="Thermal noise floor (reference element)")
    ax.set_xlabel("Baseband frequency [MHz]")
    ax.set_ylabel("PSD [dB re. thermal noise]")
    ax.set_title(f"Spectrum before and after anti-jam processing (J/S = {sc.js_db:.0f} dB)")
    ax.set_xlim(-sc.fs / 2e6, sc.fs / 2e6)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.12, 1, 1))
    fig.savefig(args.out / "fig1_psd.png", dpi=150)
    plt.close(fig)

    # -- Fig 2: null depth vs frequency + response toward the satellite ------
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4.2))
    for i, M in enumerate(taps_set):
        res = results[M][0]
        w = res.weights[-1]
        lbl = "spatial PI (M=1)" if M == 1 else f"STAP-PI M={M}"
        a1.plot(freqs / 1e6, null_depth_vs_frequency(data, w, M, ref, freqs, sc.jam_u), color=SERIES[i], label=lbl)
        a2.plot(freqs / 1e6, snr_gain_vs_frequency(data, w, M, ref, freqs, sc.sat_u), color=SERIES[i], label=lbl)
    for a in (a1, a2):
        for edge in (-sc.jam_bw_hz / 2e6, sc.jam_bw_hz / 2e6):
            a.axvline(edge, color=AXIS, lw=0.8)
        a.set_xlabel("Baseband frequency [MHz]")
        a.set_xlim(-sc.fs / 2e6, sc.fs / 2e6)
    a1.set_ylabel("Jammer gain re. reference element [dB]")
    a1.set_title("Toward the jammer: wideband null depth")
    a1.text(sc.jam_bw_hz / 2e6, a1.get_ylim()[0] + 1, " jammer\n band edge", color=MUTED, fontsize=7.5, va="bottom")
    a2.axhline(10 * np.log10(sc.n_elements), color=MUTED, lw=1.0)
    a2.text(-sc.fs / 2e6 + 0.3, 10 * np.log10(sc.n_elements) + 0.25, "2-element bound (+3 dB)", color=MUTED, fontsize=7.5)
    a2.set_ylabel("Satellite SNR gain re. reference element [dB]")
    a2.set_title("Toward the satellite: SNR preserved")
    a2.set_ylim(-2, 4.5)
    handles, labels = a1.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=len(labels), bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(args.out / "fig2_null_depth.png", dpi=150)
    plt.close(fig)

    # -- Fig 3: space-time pattern (cone angle x frequency) ------------------
    from matplotlib.colors import LinearSegmentedColormap

    cmap = LinearSegmentedColormap.from_list("blues", ["#fcfcfb"] + BLUES)
    th = np.linspace(-90, 90, 361)
    fpat = np.linspace(-12e6, 12e6, 241)
    a_all = data.element_response(cone_angle_unit_vector(th), fpat)          # (A, N, F)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10.5, 4.0), gridspec_kw={"width_ratios": [1.25, 1]})
    H = space_time_response(res_main.weights[-1], M_main, sc.fs, fpat, a_all)
    att = -db(np.abs(H) ** 2 / np.abs(a_all[:, ref, :]) ** 2)                 # attenuation re ref element
    im = a1.pcolormesh(th, fpat / 1e6, att.T, cmap=cmap, vmin=-10, vmax=70, shading="auto")
    a1.grid(False)
    for ang, lbl in ((th_j, "jammer"), (th_s, "satellite")):
        a1.axvline(ang, color=INK2, lw=0.8)
        a1.text(ang + 2, 10.8, lbl, color=INK, fontsize=8)
    a1.set_xlabel("Angle off broadside [deg]")
    a1.set_ylabel("Baseband frequency [MHz]")
    a1.set_title(f"STAP-PI (M={M_main}) attenuation vs angle and frequency")
    cb = fig.colorbar(im, ax=a1, pad=0.02)
    cb.set_label("Attenuation re. reference element [dB]", color=INK2, fontsize=8.5)
    cb.outline.set_visible(False)
    for i, (M, lbl) in enumerate(((1, "spatial PI (M=1)"), (M_main, f"STAP-PI M={M_main}"))):
        a_c = data.element_response(cone_angle_unit_vector(th), np.array([0.0, 9e6]))
        Hc = space_time_response(results[M][0].weights[-1], M, sc.fs, np.array([0.0, 9e6]), a_c)
        g = db(np.abs(Hc) ** 2 / np.abs(a_c[:, ref, :]) ** 2)
        z = 3 if M == 1 else 2                       # draw the shallow spatial-only null on top
        a2.plot(th, g[:, 0], color=SERIES[i], label=f"{lbl}, f = 0 MHz", zorder=z)
        a2.plot(th, g[:, 1], color=SERIES[i], lw=1.2, alpha=0.55, label=f"{lbl}, f = +9 MHz", zorder=z)
    for ang in (th_j, th_s):
        a2.axvline(ang, color=AXIS, lw=0.8)
    a2.set_xlabel("Angle off broadside [deg]")
    a2.set_ylabel("Gain re. reference element [dB]")
    a2.set_title("Pattern cuts")
    a2.set_ylim(-90, 15)
    a2.set_xlim(-90, 90)
    a2.legend(loc="lower right", fontsize=7.5)
    fig.tight_layout()
    fig.savefig(args.out / "fig3_pattern.png", dpi=150)
    plt.close(fig)

    # -- Fig 4: C/A correlation before / after -----------------------------
    delays = np.arange(-15, 15.01, 0.1)
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
    panels = ((f"Reference element, jammed (J/S = {sc.js_db:.0f} dB)", data.x[ref], SERIES[0], m_main["cn0_in_db"]),
              (f"STAP-PI output (M={M_main})", res_main.y, SERIES[2], m_main["cn0_out_db"]))
    for a, (lbl, sig, col, cn0) in zip(axs, panels):
        c = correlate_ca(sig, sc.fs, sc.prn, sc.doppler_hz, sc.chip_rate, sc.code_phase_chips, delays, 1e-3, start=k0)
        fl = np.mean(c[np.abs(delays) >= 3])
        a.plot(delays, c / fl, color=col, lw=1.6)
        a.set_title(lbl)
        a.set_xlabel("Code delay [chips]")
        note = f"effective C/N0 = {cn0:.1f} dB-Hz" + ("\nno correlation peak" if cn0 < 25 else "")
        a.text(0.98, 0.95, note, transform=a.transAxes, ha="right", va="top", color=INK, fontsize=9,
               bbox={"facecolor": SURFACE, "edgecolor": "none", "pad": 2.5})
    axs[0].set_ylabel("Correlation power / noise floor")
    fig.suptitle(f"GPS C/A PRN {sc.prn} correlation, 1 ms coherent x {int((sc.duration_s*sc.fs - k0)//(1e-3*sc.fs))} non-coherent",
                 x=0.01, ha="left", fontsize=10, color=INK2)
    fig.tight_layout()
    fig.savefig(args.out / "fig4_correlation.png", dpi=150)
    plt.close(fig)

    # -- Fig 5: J/S sweep -----------------------------------------------------
    js_list = np.arange(40, 121, 20 if args.quick else 10)
    sweep_taps = [1, 3, 5, 7]
    supp = {M: [] for M in sweep_taps}
    cn0o = {M: [] for M in sweep_taps}
    ideal7, cn0_in_sweep = [], []
    for js in js_list:
        d = CrpaScenario(js_db=float(js), adc_bits=adc, mismatch=not args.no_mismatch, duration_s=4e-3, seed=11).generate()
        for M in sweep_taps:
            mm = evaluate(d, StapPIProcessor(n_taps=M).process(d.x))
            supp[M].append(mm["suppression_db"])
            cn0o[M].append(mm["cn0_out_db"])
        cn0_in_sweep.append(mm["cn0_in_db"])                 # single-element figure, independent of M
        di = CrpaScenario(js_db=float(js), adc_bits=None, mismatch=not args.no_mismatch, duration_s=4e-3, seed=11).generate()
        ideal7.append(evaluate(di, StapPIProcessor(n_taps=7).process(di.x))["cn0_out_db"])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4.4))
    for i, M in enumerate(sweep_taps):
        lbl = "spatial PI (M=1)" if M == 1 else f"STAP-PI M={M}"
        a1.plot(js_list, supp[M], color=SERIES[i], marker="o", ms=4, label=lbl)
        a2.plot(js_list, cn0o[M], color=SERIES[i], marker="o", ms=4, label=lbl)
    a2.plot(js_list, ideal7, color=SERIES[3], lw=1.2, ls=(0, (4, 2)), label="STAP-PI M=7, ideal ADC")
    a2.plot(js_list, cn0_in_sweep, color=MUTED, lw=1.2, label="no anti-jam (single element)")
    a1.set_xlabel("J/S at antenna [dB]")
    a1.set_ylabel("Jammer suppression [dB]")
    a1.set_title("Jammer suppression")
    a2.set_xlabel("J/S at antenna [dB]")
    a2.set_ylabel("Effective C/N0 [dB-Hz]")
    a2.set_title(f"Post-correlation C/N0 (clean: {sc.cn0_dbhz:.0f} dB-Hz)")
    a2.set_ylim(-10, 55)
    handles, labels = a2.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.13, 1, 1))
    fig.savefig(args.out / "fig5_js_sweep.png", dpi=150)
    plt.close(fig)
    summary["js_sweep"] = {"js_db": js_list.tolist(), "suppression_db": {str(k): v for k, v in supp.items()},
                           "cn0_out_db": {str(k): v for k, v in cn0o.items()}, "cn0_out_ideal_adc_M7": ideal7,
                           "cn0_in_db": cn0_in_sweep}

    # -- Fig 6: taps sweep ----------------------------------------------------
    taps_axis = list(range(1, 12, 2 if args.quick else 1))
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    tap_sweep = {}
    for i, (mm_flag, lbl) in enumerate(((True, "with RF channel mismatch"), (False, "identical channels"))):
        d = CrpaScenario(js_db=sc.js_db, adc_bits=adc, mismatch=mm_flag, duration_s=4e-3, seed=5).generate()
        vals = [evaluate(d, StapPIProcessor(n_taps=M).process(d.x))["suppression_db"] for M in taps_axis]
        tap_sweep[lbl] = vals
        ax.plot(taps_axis, vals, color=SERIES[i], marker="o", ms=4, label=lbl)
    ax.axhline(m0["jnr_in_db"], color=MUTED, lw=1.0)
    ax.text(taps_axis[-1], m0["jnr_in_db"] + 1, "input JNR", ha="right", va="bottom", color=MUTED, fontsize=8)
    ax.set_xlabel("Taps per element, M")
    ax.set_ylabel("Jammer suppression [dB]")
    ax.set_title(f"Suppression vs. STAP taps (J/S = {sc.js_db:.0f} dB)")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(args.out / "fig6_taps.png", dpi=150)
    plt.close(fig)
    summary["taps_sweep"] = {"taps": taps_axis, **tap_sweep}

    # -- Fig 7: eigenvalues of R ---------------------------------------------
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    nf = float(np.mean(data.noise_power))
    ev = res_main.eigenvalues[-1]
    ax.plot(np.arange(1, ev.size + 1), db(ev / nf), color=SERIES[0], marker="o", ms=6, lw=1.2)
    ax.axhline(0, color=MUTED, lw=1.0)
    ax.text(ev.size, 1.5, "noise floor", ha="right", color=MUTED, fontsize=8)
    ax.set_xlabel("Eigenvalue index")
    ax.set_ylabel("Eigenvalue [dB re. noise]")
    ax.set_title(f"Space-time covariance eigenvalues (M={M_main}, D={ev.size})")
    fig.tight_layout()
    fig.savefig(args.out / "fig7_eigenvalues.png", dpi=150)
    plt.close(fig)

    (ROOT / "docs" / "results.json").write_text(json.dumps(summary, indent=2, default=float))
    print(f"\nfigures written to {args.out}  (done in {time.time()-t0:.1f} s)")
    return summary


if __name__ == "__main__":
    main()
