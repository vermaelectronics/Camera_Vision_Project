function w = stap_pi_weights(R, s, delta)
%STAP_PI_WEIGHTS Power-inversion weights  w = (R+dI)^-1 s / (s' (R+dI)^-1 s).
%   Solved with a Cholesky factorisation R + dI = L L' and two triangular
%   substitutions (no explicit matrix inverse).  The result satisfies the
%   power-inversion constraint w' * s = 1.

if nargin < 3, delta = 0; end
D = size(R, 1);
L = chol(R + delta * eye(D), 'lower');
z = L' \ (L \ s);
w = z / (s' * z);
end
