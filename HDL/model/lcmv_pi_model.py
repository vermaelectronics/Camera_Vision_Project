"""Floating-point reference model for the 2-antenna LCMV-PI CRPA core.

Compares LCMV power inversion (ACES 2019, eq. 9) with the old angle-steered
LCMV and plots the beam patterns.  Requires numpy + matplotlib.

    python3 lcmv_pi_model.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(1)


def steer(theta_deg):
    """2-element ULA, d = lambda/2: a = [1, exp(j*pi*sin(theta))]."""
    return np.array([1, np.exp(1j * np.pi * np.sin(np.deg2rad(theta_deg)))])


def snapshots(jam_deg, k=1024, jam_amp=400.0, noise_amp=2.0):
    x = noise_amp * (rng.standard_normal((2, k)) + 1j * rng.standard_normal((2, k)))
    s = jam_amp * (rng.standard_normal(k) + 1j * rng.standard_normal(k))
    return x + np.outer(steer(jam_deg), s)


def lcmv_pi(r, dl_shift=16):
    """w = R^-1 s0 / (s0^H R^-1 s0)  ->  w1 = 1, w2 = -conj(R12)/R22'."""
    r22 = r[1, 1].real + np.trace(r).real / 2**dl_shift + 1
    return np.array([1.0, -np.conj(r[0, 1]) / r22])


def lcmv_angle(gps_deg, jam_deg):
    """Old design: C = [a(gps) a(jam)], f = [1 0]  ->  w = C^-H f (R-independent)."""
    c = np.column_stack([steer(gps_deg), steer(jam_deg)])
    return np.linalg.solve(c.conj().T, np.array([1, 0]))


def pattern_db(w, grid):
    p = 20 * np.log10(np.abs([w.conj() @ steer(t) for t in grid]) + 1e-12)
    return p - p.max()


if __name__ == "__main__":
    grid = np.arange(-90, 90.001, 0.02)
    true_jam = 40.0
    x = snapshots(true_jam)
    r = x @ x.conj().T / x.shape[1]

    w_pi = lcmv_pi(r)
    w_old = lcmv_angle(30, 41)        # 1 deg jammer-angle error

    i = np.argmin(abs(grid - true_jam))
    p_pi, p_old = pattern_db(w_pi, grid), pattern_db(w_old, grid)
    print(f"LCMV-PI  gain at true jammer: {p_pi[i]:6.1f} dB")
    print(f"Old LCMV gain at true jammer: {p_old[i]:6.1f} dB (jam_angle off by 1 deg)")

    plt.figure(figsize=(9, 4.5))
    plt.plot(grid, p_pi, label="LCMV-PI (power inversion)")
    plt.plot(grid, p_old, "r--", label="Old angle-based LCMV, 1° angle error")
    plt.axvline(true_jam, color="gray", ls=":")
    plt.ylim(-100, 5); plt.xlim(-90, 90); plt.grid(alpha=.3); plt.legend()
    plt.xlabel("Angle (deg)"); plt.ylabel("Normalized gain (dB)")
    plt.title("2-antenna CRPA, jammer at 40°")
    plt.tight_layout(); plt.savefig("model_pattern.png", dpi=130)
