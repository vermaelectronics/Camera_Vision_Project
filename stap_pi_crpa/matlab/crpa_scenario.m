function [x, parts, info] = crpa_scenario(p)
%CRPA_SCENARIO Generate wideband 2-element CRPA data with a barrage jammer.
%   [X, PARTS, INFO] = CRPA_SCENARIO(P) returns the digitised N x K data X,
%   its components PARTS.signal/.jammer/.noise/.adc (X = sum of parts) and
%   INFO (.agc_gain, .noise_power, .chip_rate, .signal_power, .jammer_power).
%
%   Propagation delays and channel responses are applied per FFT bin, so the
%   inter-element phase 2 pi (fc+f) tau and the RF mismatch vary across the
%   band as in hardware (true wideband model).

if nargin < 1, p = crpa_default_params(); end
rng(p.seed);
c = 299792458;
N = p.n_elements;
K = round(p.duration * p.fs);
f = [0:ceil(K/2)-1, -floor(K/2):-1] * (p.fs / K);           % fftfreq ordering
t = (0:K-1) / p.fs;
P = crpa_positions(p);

C = 10^(p.cn0_dbhz / 10) / p.fs;                            % C = C/N0 * N0, N0 = 1/fs
J = C * 10^(p.js_db / 10);
chip_rate = 1.023e6 * (1 + p.doppler / p.fc);

code = gps_ca_code(p.prn);
chip = mod(floor(p.code_phase + t * chip_rate), 1023) + 1;
s0 = sqrt(C) * code(chip) .* exp(2i * pi * p.doppler * t + 0.3i);
S0 = fft(s0);

J0 = zeros(1, K);
if p.jammer_on
    band = abs(f - p.jam_offset) <= p.jam_bw / 2;
    nb = nnz(band);
    J0(band) = (randn(1, nb) + 1i * randn(1, nb)) / sqrt(2);
    j0 = ifft(J0);
    J0 = J0 * sqrt(J / mean(abs(j0).^2));
end

us = crpa_doa(p.sat_az, p.sat_el);
uj = crpa_doa(p.jam_az, p.jam_el);
sig = zeros(N, K); jam = zeros(N, K); noi = zeros(N, K);
for m = 1:N
    H = crpa_channel_response(p.channels(m), f, p.frontend_bw / 2, p.frontend_order);
    a_s = exp(2i * pi * (p.fc + f) * (P(m, :) * us) / c);
    a_j = exp(2i * pi * (p.fc + f) * (P(m, :) * uj) / c);
    n_t = (randn(1, K) + 1i * randn(1, K)) / sqrt(2);
    sig(m, :) = ifft(H .* a_s .* S0);
    jam(m, :) = ifft(H .* a_j .* J0);
    noi(m, :) = ifft(H .* fft(n_t));
end
analog = sig + jam + noi;

if isempty(p.adc_bits)
    g = ones(N, 1); adc = zeros(N, K); q_power = 0;
else
    rail_rms = sqrt(mean(abs(analog).^2, 2) / 2);
    g = 10^(-p.adc_backoff_db / 20) ./ rail_rms;
    scaled = bsxfun(@times, analog, g);
    delta = 2 / 2^p.adc_bits;
    top = 1 - delta / 2;
    q = @(v) min(max(delta * (floor(v / delta) + 0.5), -top), top);
    adc = (q(real(scaled)) + 1i * q(imag(scaled))) - scaled;
    q_power = delta^2 / 6;
end
parts.signal = bsxfun(@times, sig, g);
parts.jammer = bsxfun(@times, jam, g);
parts.noise  = bsxfun(@times, noi, g);
parts.adc    = adc;
x = parts.signal + parts.jammer + parts.noise + parts.adc;

info.agc_gain = g;
info.noise_power = mean(abs(parts.noise).^2, 2) + q_power;
info.chip_rate = chip_rate;
info.signal_power = C;
info.jammer_power = J;
end
