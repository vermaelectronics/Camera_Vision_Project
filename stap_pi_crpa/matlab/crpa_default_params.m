function p = crpa_default_params()
%CRPA_DEFAULT_PARAMS Default wideband barrage-jammer scenario for a 2-element CRPA.
%   Mirrors python/stap_pi/scenario.py (CrpaScenario).  Powers are referred
%   to the antenna: thermal noise has unit power over fs (N0 = 1/fs).

p.fc             = 1575.42e6;   % GPS L1 carrier [Hz]
p.fs             = 30e6;        % complex sample rate [Hz]
p.duration       = 10e-3;       % record length [s]
p.n_elements     = 2;
p.spacing_wl     = 0.5;         % element spacing [wavelengths]
p.positions      = [];          % N x 3 element positions [m] (overrides spacing)

% GNSS satellite (GPS C/A, BPSK(1))
p.prn            = 7;
p.cn0_dbhz       = 45;
p.sat_az         = 30;          % [deg] from +x towards +y
p.sat_el         = 60;          % [deg]
p.doppler        = 1250;        % [Hz]
p.code_phase     = 211.4;       % [chips]

% Wideband barrage noise jammer
p.js_db          = 95;          % jammer-to-signal ratio at the antenna [dB]
p.jam_az         = 120;
p.jam_el         = 10;
p.jam_bw         = 20e6;        % barrage bandwidth [Hz]
p.jam_offset     = 0;           % centre offset from L1 [Hz]
p.jammer_on      = true;

% Receiver front-end and digitiser
p.frontend_bw    = 24e6;        % IF filter bandwidth [Hz] (Butterworth cutoff = bw/2)
p.frontend_order = 6;
p.channels       = crpa_default_channels();
p.adc_bits       = 14;          % [] = ideal ADC
p.adc_backoff_db = 15;          % rms per rail below full scale [dB]
p.seed           = 1;
end
