function [y, W, info] = stap_pi_process(x, cfg)
%STAP_PI_PROCESS Block-adaptive STAP power-inversion anti-jam processor.
%   [Y, W, INFO] = STAP_PI_PROCESS(X, CFG) filters the N x K element data X
%   (complex baseband) and returns the 1 x K output Y, the weights W
%   ((N*M) x n_blocks) applied in each block and diagnostic INFO
%   (.loading, .eigenvalues, .s).
%
%   For each block b of CFG.block_len samples:
%     1. space-time snapshots x(n) = [x1(n) .. x1(n-M+1) x2(n) .. x2(n-M+1)].'
%     2. R_b = mean(x x'), optionally exponentially averaged
%     3. w_b = (R_b + dI)^-1 s / (s' (R_b + dI)^-1 s)   (s = unit vector on the
%        reference element / tap; d = diagonal loading)
%     4. y(n) = w' x(n) using the weight computed CFG.weight_latency blocks earlier.
%
%   See also STAP_PI_DEFAULT_CONFIG, STAP_PI_WEIGHTS, STAP_PI_APPLY.

if nargin < 2, cfg = stap_pi_default_config(); end
[N, K] = size(x);
M = cfg.n_taps;
D = N * M;
if isempty(cfg.ref_tap), ref_tap = floor((M - 1) / 2) + 1; else, ref_tap = cfg.ref_tap; end
s = zeros(D, 1);
s((cfg.ref_element - 1) * M + ref_tap) = 1;

L = cfg.block_len;
nb = ceil(K / L);
y = zeros(1, K);
W = zeros(D, nb);
info.loading = zeros(1, nb);
info.eigenvalues = zeros(D, nb);
info.s = s;
info.ref_tap = ref_tap;

hist = zeros(N, M - 1);
Rs = [];
queue = repmat(s, 1, cfg.weight_latency);       % pass-through until adapted
for b = 1:nb
    idx = (b - 1) * L + 1 : min(b * L, K);
    [X, hist] = stap_snapshots(x(:, idx), M, hist);
    Xc = X(:, 1:cfg.cov_decimation:end);
    if size(Xc, 2) >= D
        Rb = (Xc * Xc') / size(Xc, 2);
        Rb = (Rb + Rb') / 2;
        if isempty(Rs), Rs = Rb; else, Rs = cfg.forgetting * Rs + (1 - cfg.forgetting) * Rb; end
    end
    if isempty(Rs)
        w_new = s; delta = 0; ev = zeros(D, 1);
    else
        ev = sort(real(eig(Rs)), 'descend');
        if isempty(cfg.noise_power), nf = max(ev(end), realmin); else, nf = cfg.noise_power; end
        nf = max(nf, 1e-14 * real(trace(Rs)) / D);
        delta = 10^(cfg.loading_db / 10) * nf;
        w_new = stap_pi_weights(Rs, s, delta);
    end
    if cfg.weight_latency == 0
        w_use = w_new;
    else
        w_use = queue(:, 1);
        queue = [queue(:, 2:end), w_new];
    end
    y(idx) = w_use' * X;
    W(:, b) = w_use;
    info.loading(b) = delta;
    info.eigenvalues(:, b) = ev;
end
end
