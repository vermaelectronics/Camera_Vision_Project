"""
Wideband 2-element CRPA scenario generator.

Everything is generated at complex baseband around the carrier ``fc`` and is
*truly wideband*: propagation delays and RF/IF channel responses are applied
in the frequency domain, bin by bin, so the inter-element phase
``2*pi*(fc + f)*tau`` and the channel mismatch both vary across the band
exactly as they do in hardware.

Received signal at element ``m`` (frequency domain):

    X_m(f) = H_m(f) * [ a_m(f; sat) S(f) + a_m(f; jam) J(f) + N_m(f) ]

    a_m(f; u) = exp(+j 2 pi (fc + f) p_m.u / c)      plane-wave steering
    H_m(f)     = g_m e^{j phi_m} B_m(f) (1 + rho_m e^{-j 2 pi f T_m}) e^{-j 2 pi f skew_m}
                 (gain/phase imbalance, Butterworth IF filter with per-channel
                  cutoff error, an "echo" giving amplitude/group-delay ripple,
                  and a sampling-clock skew)

followed by a per-channel AGC and an ``adc_bits`` I/Q ADC.

Powers are referenced to the antenna input: thermal noise has unit power
over ``fs`` (so ``N0 = 1/fs``), the GNSS signal has ``C = C/N0 * N0`` and the
jammer ``J = C * 10^(J/S / 10)`` spread over ``jam_bw_hz`` (barrage).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "SPEED_OF_LIGHT",
    "GPS_L1_HZ",
    "CA_CHIP_RATE",
    "ChannelModel",
    "default_channel_models",
    "CrpaScenario",
    "CrpaData",
    "gps_ca_code",
    "doa_unit_vector",
    "cone_angle_unit_vector",
]

SPEED_OF_LIGHT = 299_792_458.0
GPS_L1_HZ = 1575.42e6
CA_CHIP_RATE = 1.023e6

# G2 phase-selector taps for the GPS C/A code (IS-GPS-200, Table 3-Ia)
_CA_G2_TAPS = {
    1: (2, 6), 2: (3, 7), 3: (4, 8), 4: (5, 9), 5: (1, 9), 6: (2, 10), 7: (1, 8), 8: (2, 9),
    9: (3, 10), 10: (2, 3), 11: (3, 4), 12: (5, 6), 13: (6, 7), 14: (7, 8), 15: (8, 9),
    16: (9, 10), 17: (1, 4), 18: (2, 5), 19: (3, 6), 20: (4, 7), 21: (5, 8), 22: (6, 9),
    23: (1, 3), 24: (4, 6), 25: (5, 7), 26: (6, 8), 27: (7, 9), 28: (8, 10), 29: (1, 6),
    30: (2, 7), 31: (3, 8), 32: (4, 9),
}


def gps_ca_code(prn: int) -> np.ndarray:
    """1023-chip GPS C/A Gold code for ``prn`` (1..32) as +/-1 (bit 0 -> +1)."""
    t1, t2 = _CA_G2_TAPS[prn]
    g1 = [1] * 10
    g2 = [1] * 10
    out = np.empty(1023, dtype=np.int8)
    for i in range(1023):
        chip = g1[9] ^ g2[t1 - 1] ^ g2[t2 - 1]
        out[i] = 1 - 2 * chip
        f1 = g1[2] ^ g1[9]
        f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [f1] + g1[:9]
        g2 = [f2] + g2[:9]
    return out


def doa_unit_vector(az_deg: float, el_deg: float) -> np.ndarray:
    """Unit vector *towards* a source at azimuth ``az`` (from +x towards +y) and elevation ``el``."""
    az, el = np.deg2rad(az_deg), np.deg2rad(el_deg)
    return np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])


def cone_angle_unit_vector(theta_deg) -> np.ndarray:
    """Unit vector(s) at angle ``theta`` off broadside of an x-axis array (in the x-z plane).

    For a 2-element (linear) array every direction is characterised by this
    single cone angle: ``sin(theta) = u_x``.
    """
    th = np.deg2rad(np.atleast_1d(theta_deg))
    return np.stack([np.sin(th), np.zeros_like(th), np.cos(th)], axis=-1)


@dataclass
class ChannelModel:
    """Analogue RF/IF chain of one element (lowpass-equivalent)."""

    gain_db: float = 0.0
    phase_deg: float = 0.0
    cutoff_error: float = 0.0       # relative error of the IF filter cutoff
    echo_amplitude: float = 0.0     # rho: amplitude/group-delay ripple
    echo_delay_s: float = 40e-9     # T
    skew_s: float = 0.0             # sampling-clock / cable skew

    def response(self, freqs: np.ndarray, cutoff_hz: float, order: int) -> np.ndarray:
        """Complex response ``H(f)`` on the baseband frequency grid ``freqs``."""
        wc = 2 * np.pi * cutoff_hz * (1.0 + self.cutoff_error)
        k = np.arange(order)
        poles = wc * np.exp(1j * np.pi * (2 * k + order + 1) / (2 * order))   # LHP Butterworth poles
        s = 2j * np.pi * np.asarray(freqs)
        B = np.prod((-poles[:, None]) / (s[None, :] - poles[:, None]), axis=0)
        H = (10 ** (self.gain_db / 20) * np.exp(1j * np.deg2rad(self.phase_deg)) * B
             * (1 + self.echo_amplitude * np.exp(-s * self.echo_delay_s))
             * np.exp(-s * self.skew_s))
        return H


def default_channel_models() -> list[ChannelModel]:
    """Representative mismatch between two COTS RF front-ends.

    0.6 dB gain and 7 deg phase imbalance, 3 % IF-filter cutoff difference,
    ~0.2-0.3 dB amplitude ripple and 0.15 ns skew.  On its own this mismatch
    limits a spatial-only 2-element null to roughly 25-30 dB.
    """
    return [
        ChannelModel(gain_db=0.0, phase_deg=0.0, cutoff_error=0.00, echo_amplitude=0.020, echo_delay_s=35e-9, skew_s=0.0),
        ChannelModel(gain_db=0.6, phase_deg=7.0, cutoff_error=0.03, echo_amplitude=0.035, echo_delay_s=50e-9, skew_s=0.15e-9),
    ]


@dataclass
class CrpaData:
    """Generated data set.  ``x == sum(parts.values())`` (to rounding)."""

    x: np.ndarray                    # (n_el, n) total digitised samples
    parts: dict                      # 'signal', 'jammer', 'noise', 'adc' -> (n_el, n)
    fs: float
    scenario: "CrpaScenario" = field(repr=False)
    agc_gain: np.ndarray = field(repr=False)        # (n_el,) linear amplitude gains
    noise_power: np.ndarray = field(repr=False)     # (n_el,) thermal + ADC noise power at processor input

    def element_response(self, u: np.ndarray, freqs: np.ndarray) -> np.ndarray:
        """``(A, n_el, F)`` response (steering x channel x AGC) for unit vectors ``u`` (A, 3)."""
        sc = self.scenario
        u = np.atleast_2d(u)
        P = sc.positions()
        delay = (u @ P.T) / SPEED_OF_LIGHT                                     # (A, n_el), p.u / c
        steer = np.exp(2j * np.pi * (sc.fc + freqs)[None, None, :] * delay[:, :, None])
        return steer * self.channel_response(freqs)[None]

    def channel_response(self, freqs: np.ndarray) -> np.ndarray:
        """``(n_el, F)`` RF/IF channel response including the AGC gain."""
        sc = self.scenario
        H = np.stack([ch.response(freqs, sc.frontend_bw_hz / 2, sc.frontend_order) for ch in sc.channels])
        return H * self.agc_gain[:, None]


@dataclass
class CrpaScenario:
    """Wideband barrage-jammer scenario for a 2-element CRPA."""

    fc: float = GPS_L1_HZ
    fs: float = 30e6
    duration_s: float = 10e-3
    n_elements: int = 2
    spacing_wavelengths: float = 0.5
    element_positions: np.ndarray | None = None      # (n_el, 3) [m]; overrides spacing

    # GNSS satellite (GPS C/A, BPSK(1))
    prn: int = 7
    cn0_dbhz: float = 45.0
    sat_az_deg: float = 30.0
    sat_el_deg: float = 60.0
    doppler_hz: float = 1250.0
    code_phase_chips: float = 211.4

    # Wideband barrage noise jammer
    js_db: float = 95.0                              # jammer-to-signal ratio at antenna
    jam_az_deg: float = 120.0
    jam_el_deg: float = 10.0
    jam_bw_hz: float = 20e6
    jam_offset_hz: float = 0.0
    jammer_on: bool = True

    # Receiver front-end and digitiser
    frontend_bw_hz: float = 24e6
    frontend_order: int = 6
    channels: list[ChannelModel] | None = None       # None -> default mismatch
    mismatch: bool = True                            # False -> identical ideal channels
    adc_bits: int | None = 14                        # None -> ideal (unquantised)
    adc_backoff_db: float = 15.0                     # rms per rail below full scale (15 dB: clipping
                                                     # of the Gaussian jammer at 5.6 sigma is negligible)

    seed: int = 1

    def __post_init__(self):
        if self.channels is None:
            if self.mismatch:
                base = default_channel_models()
                self.channels = [base[i % len(base)] for i in range(self.n_elements)]
            else:
                self.channels = [ChannelModel() for _ in range(self.n_elements)]
        if len(self.channels) != self.n_elements:
            raise ValueError("one ChannelModel per element required")

    # -- geometry ----------------------------------------------------------
    @property
    def wavelength(self) -> float:
        return SPEED_OF_LIGHT / self.fc

    def positions(self) -> np.ndarray:
        if self.element_positions is not None:
            return np.asarray(self.element_positions, dtype=float)
        d = self.spacing_wavelengths * self.wavelength
        return np.column_stack([np.arange(self.n_elements) * d, np.zeros(self.n_elements), np.zeros(self.n_elements)])

    @property
    def sat_u(self) -> np.ndarray:
        return doa_unit_vector(self.sat_az_deg, self.sat_el_deg)

    @property
    def jam_u(self) -> np.ndarray:
        return doa_unit_vector(self.jam_az_deg, self.jam_el_deg)

    @property
    def signal_power(self) -> float:
        return 10 ** (self.cn0_dbhz / 10) / self.fs          # C = C/N0 * N0, N0 = 1/fs

    @property
    def jammer_power(self) -> float:
        return self.signal_power * 10 ** (self.js_db / 10)

    @property
    def chip_rate(self) -> float:
        return CA_CHIP_RATE * (1 + self.doppler_hz / self.fc)   # code Doppler

    # -- generation --------------------------------------------------------
    def generate(self) -> CrpaData:
        rng = np.random.default_rng(self.seed)
        n = int(round(self.duration_s * self.fs))
        f = np.fft.fftfreq(n, 1 / self.fs)
        t = np.arange(n) / self.fs
        P = self.positions()

        # GNSS signal at the array origin
        code = gps_ca_code(self.prn)
        chip = np.floor(self.code_phase_chips + t * self.chip_rate).astype(np.int64) % 1023
        s0 = np.sqrt(self.signal_power) * code[chip] * np.exp(2j * np.pi * self.doppler_hz * t + 1j * 0.3)
        S0 = np.fft.fft(s0)

        # Barrage jammer: flat complex Gaussian spectrum over jam_bw_hz
        J0 = np.zeros(n, dtype=complex)
        if self.jammer_on:
            band = np.abs(f - self.jam_offset_hz) <= self.jam_bw_hz / 2
            J0[band] = (rng.standard_normal(band.sum()) + 1j * rng.standard_normal(band.sum())) / np.sqrt(2)
            j0 = np.fft.ifft(J0)
            J0 *= np.sqrt(self.jammer_power / np.mean(np.abs(j0) ** 2))

        H = np.stack([ch.response(f, self.frontend_bw_hz / 2, self.frontend_order) for ch in self.channels])
        steer_s = np.exp(2j * np.pi * (self.fc + f)[None, :] * (P @ self.sat_u)[:, None] / SPEED_OF_LIGHT)
        steer_j = np.exp(2j * np.pi * (self.fc + f)[None, :] * (P @ self.jam_u)[:, None] / SPEED_OF_LIGHT)

        noise_t = (rng.standard_normal((self.n_elements, n)) + 1j * rng.standard_normal((self.n_elements, n))) / np.sqrt(2)
        sig = np.fft.ifft(H * steer_s * S0[None, :], axis=1)
        jam = np.fft.ifft(H * steer_j * J0[None, :], axis=1)
        noi = np.fft.ifft(H * np.fft.fft(noise_t, axis=1), axis=1)
        analog = sig + jam + noi

        # Per-channel AGC + I/Q ADC
        if self.adc_bits is None:
            gain = np.ones(self.n_elements)
            adc = np.zeros_like(analog)
            q_power = 0.0
        else:
            rail_rms = np.sqrt(np.mean(np.abs(analog) ** 2, axis=1) / 2)
            gain = 10 ** (-self.adc_backoff_db / 20) / rail_rms
            scaled = analog * gain[:, None]
            delta = 2.0 / 2 ** self.adc_bits
            top = 1 - delta / 2
            quant = lambda v: np.clip(delta * (np.floor(v / delta) + 0.5), -top, top)  # noqa: E731
            digit = quant(scaled.real) + 1j * quant(scaled.imag)
            adc = digit - scaled
            q_power = delta ** 2 / 6                                            # I + Q
        g = gain[:, None]
        parts = {"signal": sig * g, "jammer": jam * g, "noise": noi * g, "adc": adc}
        x = parts["signal"] + parts["jammer"] + parts["noise"] + parts["adc"]
        noise_power = np.mean(np.abs(parts["noise"]) ** 2, axis=1) + q_power
        return CrpaData(x=x, parts=parts, fs=self.fs, scenario=self, agc_gain=gain, noise_power=noise_power)
