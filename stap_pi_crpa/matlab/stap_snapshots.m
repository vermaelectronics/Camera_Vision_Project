function [X, hist] = stap_snapshots(x, M, hist)
%STAP_SNAPSHOTS Space-time snapshot matrix of an N-element, M-tap array.
%   [X, HIST] = STAP_SNAPSHOTS(X_IN, M, HIST) returns the (N*M) x K matrix
%   with X((m-1)*M + k + 1, n) = x_in(m, n - k), k = 0..M-1 (element-major
%   ordering).  HIST holds the last M-1 samples of the previous block
%   (N x (M-1), zeros if omitted) and the updated history is returned, so a
%   stream can be processed block by block.

[N, K] = size(x);
if nargin < 3 || isempty(hist)
    hist = zeros(N, M - 1);
end
xe = [hist, x];
X = zeros(N * M, K);
for m = 1:N
    for k = 0:M-1
        X((m - 1) * M + k + 1, :) = xe(m, (M - k):(M - k + K - 1));
    end
end
hist = xe(:, end - M + 2:end);
end
