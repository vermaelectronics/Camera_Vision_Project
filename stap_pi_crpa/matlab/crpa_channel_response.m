function H = crpa_channel_response(ch, f, cutoff, order)
%CRPA_CHANNEL_RESPONSE Lowpass-equivalent response of one RF/IF chain.
%   H(f) = g e^{j phi} B(f) (1 + rho e^{-j 2 pi f T}) e^{-j 2 pi f skew}
%   with B an analogue Butterworth lowpass (cutoff*(1+cutoff_error)).
wc = 2 * pi * cutoff * (1 + ch.cutoff_error);
k = (0:order-1).';
poles = wc * exp(1i * pi * (2 * k + order + 1) / (2 * order));
s = 2i * pi * f(:).';
B = prod(bsxfun(@rdivide, -poles, bsxfun(@minus, s, poles)), 1);
H = 10^(ch.gain_db / 20) * exp(1i * pi * ch.phase_deg / 180) .* B ...
    .* (1 + ch.echo_amp * exp(-s * ch.echo_delay)) .* exp(-s * ch.skew);
end
