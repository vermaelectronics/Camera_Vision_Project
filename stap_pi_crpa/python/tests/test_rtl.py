"""Bit-exact co-simulation of rtl/stap_pi_filter.v (skipped without Icarus Verilog)."""

import pathlib
import shutil
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from rtl_cosim import run_cosim  # noqa: E402
from stap_pi.fixed_point import quantize_input, quantize_weights, stap_filter_fixed  # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("iverilog") is None, reason="Icarus Verilog not installed")


@pytest.mark.parametrize("n_taps,gap,out_shift", [(7, 0, 10), (5, 1, 12), (1, 0, 16)])
def test_rtl_bit_exact_and_fixed_point_loss(n_taps, gap, out_shift):
    r = run_cosim(n_taps=n_taps, block_len=512, n_blocks=4, out_shift=out_shift, gap=gap)
    assert r["mismatches"] == 0
    if n_taps > 1:                                  # adaptive output is at the noise floor
        assert r["fixed_vs_float_error_db_re_noise"] < -20


def test_fixed_point_model_rounding_and_saturation():
    x = np.array([[0.5 + 0.25j, -1.0 + 0.0j, 0.0 + 0.0j]])
    xr, xi = quantize_input(np.vstack([x, x]), 16)
    assert xr[0, 0] == 16384 and xi[0, 0] == 8192 and xr[0, 1] == -32768
    wr, wi = quantize_weights(np.array([[3.0 + 0.0j, 1.0 - 0.5j]]), 18, 16)
    assert wr[0, 0] == (1 << 17) - 1                # saturates at +2 - 2^-16
    assert wr[0, 1] == 65536 and wi[0, 1] == -32768
    # single element, single tap, unity weight: y = round(x * 2^16 / 2^s)
    yr, yi = stap_filter_fixed(np.array([[3, -3, 5]]), np.array([[0, 0, -5]]),
                               np.array([[65536]]), np.array([[0]]), 8, 1, out_shift=17)
    assert list(yr) == [2, -1, 3] and list(yi) == [0, 0, -2]   # round half up: 1.5->2, -1.5->-1
