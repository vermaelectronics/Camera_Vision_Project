function P = crpa_positions(p)
%CRPA_POSITIONS Element positions (N x 3, metres).
if ~isempty(p.positions)
    P = p.positions;
else
    lambda = 299792458 / p.fc;
    P = [(0:p.n_elements-1).' * p.spacing_wl * lambda, zeros(p.n_elements, 2)];
end
end
