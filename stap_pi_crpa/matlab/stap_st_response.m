function H = stap_st_response(w, M, fs, f, a)
%STAP_ST_RESPONSE Space-time filter response for an incident wave.
%   H = STAP_ST_RESPONSE(W, M, FS, F, A) with A the N x numel(F) element
%   response (steering x channel) returns
%   H(f) = sum_m sum_k conj(w(m,k)) a_m(f) exp(-j 2 pi f k / fs).
N = numel(w) / M;
Wc = reshape(conj(w(:)), M, N).';                  % N x M
tap = exp(-2i * pi * (0:M-1).' * f(:).' / fs);    % M x F
H = sum((Wc * tap) .* a, 1);
end
