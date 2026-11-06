function ch = crpa_default_channels(ideal)
%CRPA_DEFAULT_CHANNELS RF/IF channel models of the two elements.
%   Representative COTS mismatch: 0.6 dB / 7 deg imbalance, 3 % IF-filter
%   cutoff difference, ~0.2-0.3 dB echo ripple and 0.15 ns skew.  On its own
%   this limits a spatial-only 2-element null to about 25 dB.
%   CRPA_DEFAULT_CHANNELS(true) returns two identical, ideal channels.
if nargin < 1, ideal = false; end
ch = struct('gain_db', {0, 0.6}, 'phase_deg', {0, 7}, 'cutoff_error', {0, 0.03}, ...
            'echo_amp', {0.020, 0.035}, 'echo_delay', {35e-9, 50e-9}, 'skew', {0, 0.15e-9});
if ideal
    ch = struct('gain_db', {0, 0}, 'phase_deg', {0, 0}, 'cutoff_error', {0, 0}, ...
                'echo_amp', {0, 0}, 'echo_delay', {0, 0}, 'skew', {0, 0});
end
end
