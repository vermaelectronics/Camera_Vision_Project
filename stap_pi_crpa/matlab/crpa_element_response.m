function a = crpa_element_response(p, agc_gain, u, f)
%CRPA_ELEMENT_RESPONSE N x numel(F) response (steering x channel x AGC) for direction U (3x1).
c = 299792458;
P = crpa_positions(p);
a = zeros(p.n_elements, numel(f));
for m = 1:p.n_elements
    H = crpa_channel_response(p.channels(m), f, p.frontend_bw / 2, p.frontend_order);
    a(m, :) = agc_gain(m) * H .* exp(2i * pi * (p.fc + f(:).') * (P(m, :) * u) / c);
end
end
