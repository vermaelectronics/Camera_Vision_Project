function m = crpa_evaluate(p, parts, info, W, cfg)
%CRPA_EVALUATE Anti-jam figures of merit of a STAP-PI run.
%   M = CRPA_EVALUATE(P, PARTS, INFO, W, CFG) pushes every component of the
%   data through the weight schedule W (linearity) and returns, in dB/dB-Hz:
%     .jnr_in, .jnr_out      jammer-to-noise ratio at the reference element / output
%     .suppression           jammer power in (reference element) / jammer power out
%     .noise_gain            output / input noise power
%     .cn0_ref               effective C/N0, bare reference element, no jammer
%     .cn0_in                effective C/N0, bare reference element, jammed
%     .cn0_out               effective C/N0, STAP-PI output, jammed
%   The first (weight_latency+1) blocks (start-up) are excluded.

M = cfg.n_taps; L = cfg.block_len; ref = cfg.ref_element;
k0 = (cfg.weight_latency + 1) * L + 1;
names = {'signal', 'jammer', 'noise', 'adc'};
for i = 1:numel(names)
    n = names{i};
    yo.(n) = stap_pi_apply(parts.(n), W, L, M);
    yo.(n) = yo.(n)(k0:end);
    xi.(n) = parts.(n)(ref, k0:end);
end
pw = @(v) mean(abs(v).^2);
n_in = pw(xi.noise) + pw(xi.adc);
n_out = pw(yo.noise) + pw(yo.adc);
m.jnr_in = 10 * log10(pw(xi.jammer) / n_in);
m.jnr_out = 10 * log10(pw(yo.jammer) / n_out);
m.suppression = 10 * log10(pw(xi.jammer) / pw(yo.jammer));
m.noise_gain = 10 * log10(n_out / n_in);
m.cn0_ref = effective_cn0(p, info, xi.signal, xi.noise, k0);
m.cn0_in = effective_cn0(p, info, xi.signal, xi.jammer + xi.noise + xi.adc, k0);
m.cn0_out = effective_cn0(p, info, yo.signal, yo.jammer + yo.noise + yo.adc, k0);
end

function cn0 = effective_cn0(p, info, sig, interf, k0)
% Post-correlation C/N0: |coherent correlation of the satellite component
% with the ideal C/A replica|^2 over the J+N density seen through the C/A
% spectrum (spectral-separation form).
t = (k0 - 1 + (0:numel(sig) - 1)) / p.fs;
base = sig .* exp(-2i * pi * p.doppler * t);
code = gps_ca_code(p.prn);
best = 0;
for d = -0.30:0.01:0.60
    chip = mod(floor(p.code_phase - d + t * info.chip_rate), 1023) + 1;
    best = max(best, abs(sum(base .* code(chip)))^2);
end
c_eff = best / numel(sig)^2;
[f, phi] = welch_psd(interf, p.fs, 1024);
tc = 1 / info.chip_rate;
g = tc * sinc_(f * tc).^2;
df = f(2) - f(1);
g = g / (sum(g) * df);
cn0 = 10 * log10(c_eff / (sum(phi .* g) * df));
end

function y = sinc_(x)
y = ones(size(x));
nz = x ~= 0;
y(nz) = sin(pi * x(nz)) ./ (pi * x(nz));
end
