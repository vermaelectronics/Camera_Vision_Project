function [f, P] = welch_psd(x, fs, nfft)
%WELCH_PSD Two-sided Welch PSD (Hann window, 50 % overlap), fftshifted axis.
%   Toolbox-free; matches python/stap_pi/metrics.py:psd.
x = x(:).';
w = 0.5 - 0.5 * cos(2 * pi * (0:nfft-1) / (nfft - 1));
step = nfft / 2;
nseg = floor((numel(x) - nfft) / step) + 1;
acc = zeros(1, nfft);
for i = 0:nseg-1
    acc = acc + abs(fft(x(i * step + (1:nfft)) .* w)).^2;
end
P = fftshift(acc / (nseg * fs * sum(w.^2)));
f = (-nfft/2:nfft/2-1) * fs / nfft;
end
