"""Unit and end-to-end tests for the STAP-PI 2-element CRPA model.

Run from ``stap_pi_crpa/python``:  ``python3 -m pytest -q tests``
"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from stap_pi import (  # noqa: E402
    CrpaScenario,
    StapPIConfig,
    StapPIProcessor,
    apply_weight_schedule,
    constraint_vector,
    evaluate,
    gps_ca_code,
    pi_weights,
    sample_covariance,
    space_time_response,
    space_time_snapshots,
)


# ---------------------------------------------------------------- building blocks
def test_ca_code_prn1_first_chips_and_gold_autocorrelation():
    c = gps_ca_code(1)
    bits = "".join("0" if v > 0 else "1" for v in c[:10])
    assert bits == "1100100000"           # IS-GPS-200: first 10 chips of PRN 1 = octal 1440
    ac = np.array([np.dot(c, np.roll(c, k)) for k in range(1, 1023)])
    assert set(np.unique(ac)) <= {-65, -1, 63}   # three-valued Gold-code autocorrelation


def test_snapshot_layout_and_history():
    rng = np.random.default_rng(0)
    x = rng.standard_normal((2, 50)) + 1j * rng.standard_normal((2, 50))
    M = 4
    X = space_time_snapshots(x, M)
    for m in range(2):
        for k in range(M):
            for n in range(k, 50):
                assert X[m * M + k, n] == x[m, n - k]
            assert np.all(X[m * M + k, :k] == 0)
    # splitting the data and carrying the history gives identical snapshots
    X1 = space_time_snapshots(x[:, :20], M)
    X2 = space_time_snapshots(x[:, 20:], M, history=x[:, 20 - (M - 1):20])
    assert np.allclose(np.hstack([X1, X2]), X)


def test_constraint_vector_centre_tap():
    s = constraint_vector(2, 7)
    assert s[3] == 1 and np.count_nonzero(s) == 1
    s = constraint_vector(2, 5, ref_element=1, ref_tap=0)
    assert s[5] == 1


def test_pi_weights_are_the_constrained_minimum():
    rng = np.random.default_rng(1)
    D = 10
    A = rng.standard_normal((D, 4 * D)) + 1j * rng.standard_normal((D, 4 * D))
    R = sample_covariance(A) + 1e3 * np.outer(A[:, 0], A[:, 0].conj())
    s = constraint_vector(2, 5)
    w = pi_weights(R, s)
    assert np.isclose(np.vdot(w, s), 1.0)                       # w^H s = 1
    w_ref = np.linalg.solve(R, s)
    w_ref /= np.vdot(s, w_ref)
    assert np.allclose(w, w_ref)
    p0 = np.real(np.vdot(w, R @ w))
    for _ in range(20):                                         # any feasible perturbation costs power
        d = rng.standard_normal(D) + 1j * rng.standard_normal(D)
        d -= s * np.vdot(s, d)                                  # keep d^H s = 0
        w2 = w + 0.1 * d
        assert np.real(np.vdot(w2, R @ w2)) >= p0


def test_space_time_response_matches_time_domain_tone():
    fs, M, f = 30e6, 5, 3.3e6
    rng = np.random.default_rng(2)
    w = rng.standard_normal(2 * M) + 1j * rng.standard_normal(2 * M)
    a = np.array([1.0, 0.7 * np.exp(1j * 0.9)])                 # element amplitudes/phases at f
    n = np.arange(400)
    x = a[:, None] * np.exp(2j * np.pi * f * n / fs)[None, :]
    y = apply_weight_schedule(x, w[None, :], 400, M)
    H = space_time_response(w, M, fs, np.array([f]), a[:, None])[0]
    assert np.allclose(y[M:], H * np.exp(2j * np.pi * f * n[M:] / fs))


def test_streaming_equals_one_shot():
    data = CrpaScenario(duration_s=1.2e-3, js_db=80, seed=3).generate()
    cfg = StapPIConfig(n_taps=5, block_len=2048)
    y_once = StapPIProcessor(cfg).process(data.x).y
    p = StapPIProcessor(cfg)
    cut = 4 * 2048
    y_parts = np.concatenate([p.process(data.x[:, :cut]).y, p.process(data.x[:, cut:]).y])
    assert np.allclose(y_once, y_parts)


def test_linearity_decomposition():
    data = CrpaScenario(duration_s=1e-3, js_db=90, seed=4).generate()
    res = StapPIProcessor(n_taps=5, block_len=2048).process(data.x)
    total = sum(res.apply(v) for v in data.parts.values())
    assert np.allclose(total, res.y, atol=1e-12 * np.max(np.abs(res.y)))


def test_generator_matches_analytic_array_response():
    """The jammer that the generator synthesises obeys the steering x channel model used for the plots."""
    sc = CrpaScenario(duration_s=0.5e-3, js_db=90, adc_bits=None, seed=5)
    data = sc.generate()
    J = np.fft.fft(data.parts["jammer"], axis=1)
    f = np.fft.fftfreq(J.shape[1], 1 / sc.fs)
    band = np.abs(f) < 0.45 * sc.jam_bw_hz
    a = data.element_response(sc.jam_u[None, :], f[band])[0]
    assert np.allclose(J[1, band] / J[0, band], a[1] / a[0], rtol=1e-6, atol=1e-9)


# ---------------------------------------------------------------- end to end
@pytest.fixture(scope="module")
def jammed():
    return CrpaScenario(js_db=95.0, duration_s=4e-3, seed=21).generate()


def test_spatial_only_pi_fails_against_wideband_jammer(jammed):
    m = evaluate(jammed, StapPIProcessor(n_taps=1).process(jammed.x))
    assert m["suppression_db"] < 30            # limited by dispersion + channel mismatch
    assert m["jnr_out_db"] > 30


def test_stap_pi_suppresses_wideband_barrage_jammer(jammed):
    m = evaluate(jammed, StapPIProcessor(n_taps=7).process(jammed.x))
    assert m["jnr_in_db"] > 60
    assert m["suppression_db"] > 65            # jammer driven below the noise floor
    assert m["jnr_out_db"] < -5
    assert m["cn0_out_db"] > m["cn0_ref_db"] - 1.0   # satellite kept (no more than 1 dB loss)
    assert m["cn0_improvement_db"] > 60
    assert abs(m["noise_gain_db"]) < 6


def test_more_taps_more_suppression(jammed):
    s = [evaluate(jammed, StapPIProcessor(n_taps=M).process(jammed.x))["suppression_db"] for M in (1, 3, 5, 7)]
    assert all(b > a for a, b in zip(s, s[1:]))


def test_no_jammer_no_degradation():
    data = CrpaScenario(jammer_on=False, duration_s=4e-3, seed=8).generate()
    m = evaluate(data, StapPIProcessor(n_taps=7).process(data.x))
    assert m["cn0_out_db"] > m["cn0_ref_db"] - 1.0


def test_causal_latency_and_decimated_covariance_still_work(jammed):
    cfg = StapPIConfig(n_taps=7, block_len=4096, cov_decimation=4, weight_latency=2, forgetting=0.5)
    m = evaluate(jammed, StapPIProcessor(cfg).process(jammed.x))
    assert m["suppression_db"] > 60
