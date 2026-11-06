function cfg = stap_pi_default_config()
%STAP_PI_DEFAULT_CONFIG Default STAP-PI processor configuration (2 elements).
cfg.n_elements     = 2;      % antenna elements
cfg.n_taps         = 7;      % FIR taps per element (1 = spatial-only PI)
cfg.ref_element    = 1;      % constrained element (1-based)
cfg.ref_tap        = [];     % constrained tap (1-based), [] = centre tap
cfg.block_len      = 4096;   % samples per weight update
cfg.cov_decimation = 1;      % use every d-th snapshot for R
cfg.forgetting     = 0;      % R_b = f*R_(b-1) + (1-f)*R_block
cfg.loading_db     = 3;      % diagonal loading re. noise floor estimate [dB]
cfg.noise_power    = [];     % calibrated noise power ([] = use min eigenvalue)
cfg.weight_latency = 1;      % blocks between estimating and applying weights
end
