%STAP_PI_DEMO 2-element CRPA, STAP power inversion vs. a wideband barrage jammer.
%
%   Generates the reference scenario (GPS L1 C/A satellite, 20 MHz barrage
%   jammer at J/S = 95 dB, 24 MHz front-end with RF channel mismatch, 14-bit
%   ADC), runs spatial-only PI (M = 1) and STAP-PI (M = 3,5,7,9), prints the
%   figures of merit and plots spectrum, wideband null depth and the
%   space-time pattern.  Runs in MATLAB (no toolboxes) and GNU Octave.
%
%   Set  make_plots = false  before calling to skip the figures.

if ~exist('make_plots', 'var'), make_plots = true; end

p = crpa_default_params();
[x, parts, info] = crpa_scenario(p);

taps = [1 3 5 7 9];
M_main = 7;
fprintf('STAP-PI 2-element CRPA - wideband barrage jammer, J/S = %.0f dB\n', p.js_db);
fprintf('%-20s %8s %8s %8s %9s %9s\n', 'processor', 'supp', 'JNR in', 'JNR out', 'C/N0 in', 'C/N0 out');
fprintf('%-20s %8s %8s %8s %9s %9s\n', '', '[dB]', '[dB]', '[dB]', '[dB-Hz]', '[dB-Hz]');
Wsave = cell(1, numel(taps));
ysave = cell(1, numel(taps));
for i = 1:numel(taps)
    cfg = stap_pi_default_config();
    cfg.n_taps = taps(i);
    [y, W] = stap_pi_process(x, cfg);
    m = crpa_evaluate(p, parts, info, W, cfg);
    if taps(i) == 1, name = 'spatial PI (M=1)'; else, name = sprintf('STAP-PI (M=%d)', taps(i)); end
    fprintf('%-20s %8.1f %8.1f %8.1f %9.1f %9.1f\n', name, m.suppression, m.jnr_in, m.jnr_out, m.cn0_in, m.cn0_out);
    Wsave{i} = W;
    ysave{i} = y;
end
fprintf('clean single-element C/N0 (no jammer): %.1f dB-Hz\n', m.cn0_ref);

if make_plots
    cfg = stap_pi_default_config();
    k0 = (cfg.weight_latency + 1) * cfg.block_len + 1;
    i1 = find(taps == 1); iM = find(taps == M_main);

    % Spectrum before / after
    [f, Pn] = welch_psd(parts.noise(1, k0:end), p.fs, 1024);
    nref = median(Pn(abs(f) < 5e6));
    [~, Px] = welch_psd(x(1, k0:end), p.fs, 1024);
    [~, P1] = welch_psd(ysave{i1}(k0:end), p.fs, 1024);
    [~, PM] = welch_psd(ysave{iM}(k0:end), p.fs, 1024);
    figure;
    plot(f / 1e6, 10 * log10([Px; P1; PM; Pn] / nref), 'LineWidth', 1.5); grid on;
    xlabel('Baseband frequency [MHz]'); ylabel('PSD [dB re. thermal noise]');
    legend('reference element (jammed)', 'spatial PI (M=1)', sprintf('STAP-PI (M=%d)', M_main), 'thermal noise');
    title(sprintf('Spectrum before / after anti-jam (J/S = %.0f dB)', p.js_db));

    % Null depth toward the jammer vs frequency
    fr = linspace(-p.fs / 2, p.fs / 2, 601);
    uj = crpa_doa(p.jam_az, p.jam_el);
    aj = crpa_element_response(p, info.agc_gain, uj, fr);
    figure; hold on; grid on;
    for i = 1:numel(taps)
        H = stap_st_response(Wsave{i}(:, end), taps(i), p.fs, fr, aj);
        plot(fr / 1e6, 20 * log10(abs(H) ./ abs(aj(1, :))), 'LineWidth', 1.5);
    end
    xlabel('Baseband frequency [MHz]'); ylabel('Jammer gain re. reference element [dB]');
    legend(arrayfun(@(M) sprintf('M=%d', M), taps, 'UniformOutput', false), 'Location', 'southoutside', 'Orientation', 'horizontal');
    title('Wideband null depth toward the jammer');

    % Space-time pattern (cone angle off broadside x frequency)
    th = -90:0.5:90; fp = linspace(-12e6, 12e6, 121);
    A = zeros(numel(fp), numel(th));
    for k = 1:numel(th)
        u = [sind(th(k)); 0; cosd(th(k))];
        a = crpa_element_response(p, info.agc_gain, u, fp);
        A(:, k) = (-20 * log10(abs(stap_st_response(Wsave{iM}(:, end), M_main, p.fs, fp, a)) ./ abs(a(1, :)))).';
    end
    figure;
    imagesc(th, fp / 1e6, A, [-10 70]); axis xy; colorbar;
    xlabel('Angle off broadside [deg]'); ylabel('Baseband frequency [MHz]');
    title(sprintf('STAP-PI (M=%d) attenuation re. reference element [dB]', M_main));
end
