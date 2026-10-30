#!/usr/bin/env python3
"""
Measures what matters for a GNSS anti-jam front end, on any complex stream:

  * output power (to compute RMS suppression)
  * the residual jammer-to-noise ratio
  * the C/N0 of the GNSS satellite measured by a real correlator
    (1 ms coherent blocks, code + Doppler wipe-off, peak vs. off-peak lags)

The C/N0 is the figure that decides whether the receiver locks. Typical
receivers acquire at >= 33-35 dB-Hz and track down to ~28-30 dB-Hz.
"""
import numpy as np

FS = 18.0e6
CHIP = 1.023e6
BLK = 18000            # 1 ms


def replica(code, dop, t):
    chip_idx = (np.floor(t * CHIP / FS).astype(int)) % 1023
    return code[chip_idx] * np.exp(2j * np.pi * dop * t / FS)


def cn0(y, t0, code, dop, max_lag=80):
    """y[k] is the stream sample with absolute time index t0+k."""
    nblk = (len(y) - max_lag - 2000) // BLK
    if nblk < 2:
        return float("nan"), 0
    lags = list(range(0, max_lag)) + list(range(400, 1600, 7))
    acc = np.zeros(len(lags))
    for b in range(nblk):
        k = b * BLK
        t = np.arange(t0 + k, t0 + k + BLK)
        r = np.conj(replica(code, dop, t))
        for i, L in enumerate(lags):
            acc[i] += np.abs(np.dot(y[k + L:k + L + BLK], r)) ** 2
    acc /= nblk
    pk = int(np.argmax(acc[:max_lag]))
    pn = np.mean(acc[max_lag:])
    snr = (acc[pk] - pn) / pn
    if snr <= 0:
        return -99.0, pk
    return 10 * np.log10(snr) + 30.0, lags[pk]


def report(label, y, t0, meta, settle):
    y = y[settle:]
    y = y - np.mean(y)          # a receiver's own DC/AGC handling
    t0 = t0 + settle
    p = np.mean(np.abs(y) ** 2)
    c, lag = cn0(y, t0, meta["code"], float(meta["dop"]))
    return {"label": label, "power": p, "cn0": c, "lag": lag}
