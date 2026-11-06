"""Cross-check the MATLAB/Octave port against the Python reference (skipped without Octave)."""

import pathlib
import shutil
import subprocess
import sys

import numpy as np
import pytest
from scipy.io import loadmat, savemat

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from stap_pi import CrpaScenario, StapPIConfig, StapPIProcessor, gps_ca_code  # noqa: E402
from stap_pi.scenario import default_channel_models  # noqa: E402

MATLAB_DIR = pathlib.Path(__file__).resolve().parents[2] / "matlab"
pytestmark = pytest.mark.skipif(shutil.which("octave") is None, reason="GNU Octave not installed")


def _octave(code: str, cwd: pathlib.Path):
    subprocess.run(["octave", "--no-gui", "--quiet", "--eval", code], cwd=cwd, check=True,
                   capture_output=True, timeout=600)


def test_matlab_processor_matches_python(tmp_path):
    data = CrpaScenario(duration_s=1.5e-3, js_db=95, seed=9).generate()
    cfg = StapPIConfig(n_taps=7, block_len=4096, forgetting=0.3, weight_latency=1)
    res = StapPIProcessor(cfg).process(data.x)

    f = np.linspace(-15e6, 15e6, 31)
    savemat(tmp_path / "in.mat", {"x": data.x, "f": f})
    _octave(
        f"addpath('{MATLAB_DIR}'); load('{tmp_path / 'in.mat'}');"
        "cfg = stap_pi_default_config(); cfg.n_taps = 7; cfg.block_len = 4096; cfg.forgetting = 0.3;"
        "[y, W] = stap_pi_process(x, cfg);"
        "ch = crpa_default_channels(); H = [crpa_channel_response(ch(1), f, 12e6, 6); crpa_channel_response(ch(2), f, 12e6, 6)];"
        "c1 = gps_ca_code(1); c7 = gps_ca_code(7);"
        f"save('-v7', '{tmp_path / 'out.mat'}', 'y', 'W', 'H', 'c1', 'c7');",
        tmp_path,
    )
    out = loadmat(tmp_path / "out.mat")
    scale = np.max(np.abs(res.y))
    assert np.allclose(out["y"].ravel(), res.y, atol=1e-9 * scale)
    assert np.allclose(out["W"].T, res.weights, atol=1e-9)
    H_py = np.stack([ch.response(f, 12e6, 6) for ch in default_channel_models()])
    assert np.allclose(out["H"], H_py, atol=1e-12)
    assert np.array_equal(out["c1"].ravel(), gps_ca_code(1))
    assert np.array_equal(out["c7"].ravel(), gps_ca_code(7))
