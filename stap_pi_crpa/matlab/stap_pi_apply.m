function y = stap_pi_apply(x, W, block_len, M)
%STAP_PI_APPLY Filter X with a per-block weight schedule W (column b on block b).
%   Used to push a single signal component (jammer, noise, satellite)
%   through exactly the weights the adaptive processor applied.
[N, K] = size(x);
y = zeros(1, K);
hist = zeros(N, M - 1);
for b = 1:size(W, 2)
    idx = (b - 1) * block_len + 1 : min(b * block_len, K);
    if isempty(idx), break; end
    [X, hist] = stap_snapshots(x(:, idx), M, hist);
    y(idx) = W(:, b)' * X;
end
end
