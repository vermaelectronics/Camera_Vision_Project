/**************************************************************************//**
 *   @file   command.c
 *   @brief  Implementation of AD9361 Command Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *******************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "command.h"
#include "console.h"
#include "ad9361_api.h"
#include "axi_dac_core.h"
//#include "platform.h"
#include "parameters.h"
#include "app_config.h"
/* GNSS-CRPA MOD-6 */
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_smi.h"         /* MOD-25: SMI-MVDR STAP */
#include "gnss_dash.h"        /* MOD-25: gnss_check? / gnss_cmds? / colours */
#include "gnss_report.h"      /* MOD-26: gnss_report?, events */
#include "gnss_mvdr_steer.h"  /* MOD-26: operator steering modules */
#include "gnss_mvdr_auto.h"
#include <math.h>              /* NAN */
/* GNSS-CRPA MOD-7 */
#include "gnss_txdma.h"
#include "axi_dmac.h"
/* GNSS-CRPA MOD-10 */
#include "gnss_info.h"
#include "gnss_config.h"     /* MOD-15: radio plan + boot behaviour */
#include "gnss_autostart.h"  /* MOD-15: automatic MVDR start + health */
#include "no_os_delay.h"
#include "gnss_ant.h"        /* LVDS / ant board: gnss_tx, pass, stap, menu */
#include "gnss_quiet.h"     /* LVDS console: engine output captured, see gnss_quiet.h */
#define GNSS_QUIET_THIS_FILE
#undef  printf
#define printf        gnss_qprintf
#undef  console_print
#define console_print gnss_qprintf

/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
command cmd_list[] = {
	/* ---- LVDS / ant board console (gnss_ant.c). Entries before
	 * CMD_ANT_END run with the engine output captured (short pages);
	 * everything after runs as in MVDR2 and prints its full text. ---- */
	{"?", "The main menu (also: Enter).", "", get_menu},
	{"menu?", "The main menu (also: Enter).", "", get_menu},
	{"help?", "What the main commands do.", "", get_ant_help},
	{"gnss_tx=", "1 = TX1 on (muted), 0 = everything off, TX1 silent.", "gnss_tx=1", set_gnss_tx},
	{"gnss_tx?", "Is TX1 transmitting? (read from the hardware)", "", get_ant_gnss_tx},
	{"pass=", "1 = raw RX1 on TX1 (gnss_tx=1, stap=0), 0 = mute.", "pass=1", set_pass},
	{"stap=", "1 = anti-jam (gnss_tx=1, pass=1), 0 = raw RX1.", "stap=1", set_stap},
	{"gnss_status?", "Radio set-up (LO, BW, rate, gain mode, gains, attenuation) + state.", "", get_ant_gnss_status},
	{"gnss_jam?", "Jammer: power RX1/RX2, phase, angle, J/N, SUPPRESSION, cost.", "", get_ant_jam},
	{"status?", "Everything on one page.", "", get_ant_status},
	{"rx?", "RX gain control page.", "", get_rx_info},
	{"stap?", "Anti-jam page.", "", get_stap},
	{"selftest?", "What limits suppression, J/N and noise.", "", get_selftest},
	{"band?", "Radio presets.", "", get_band},
	{"band=", "Select a radio preset.", "band=3", set_band},
	{"tx1_att=", "TX1 operating attenuation, mdB.", "tx1_att=34000", set_tx1_att},
	{"agc=", "1 = AUTO RX gain, 0 = MANUAL.", "agc=1", set_agc},
	{"gnss_agcstep=", "AGC rise with a jammer: dB per step, s between steps.", "gnss_agcstep=1,1", set_agcstep},
	{"rxgain=", "Manual RX gain, dB (both channels).", "rxgain=50", set_rxgain},
	{"smi=", "SMI window 2^K, loading A.", "smi=8,3", set_smi},
	{"smiavg=", "Covariance averaging 0..4.", "smiavg=2", set_smiavg},
	{"taps?", "STAP taps per antenna in use.", "", get_taps},
	{"taps=", "STAP taps per antenna, 1..16 (live, no restart).", "taps=8", set_taps},
	{"gnss_kpi?", "All figures on one line (logger).", "", get_ant_kpi},
#define CMD_ANT_END 24
	/* ---- MVDR2 console: AD9361 vendor commands and every gnss_ command,
	 * unchanged. gnss_status_full?, gnss_jam_est?, gnss_kpi_full? and
	 * commands? are the MVDR2 gnss_status?, gnss_jam?, gnss_kpi?, help?. ---- */
	{"commands?", "Displays all available commands.", "", get_help},
	{"register?", "Gets the specified register value.", "", get_register},
	{"tx_lo_freq?", "Gets current TX LO frequency [MHz].", "", get_tx_lo_freq},
	{"tx_lo_freq=", "Sets the TX LO frequency [MHz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_tx_lo_freq},
	{"tx_samp_freq?", "Gets current TX sampling frequency [Hz].", "", get_tx_samp_freq},
	{"tx_samp_freq=", "Sets the TX sampling frequency [Hz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_tx_samp_freq},
	{"tx_rf_bandwidth?", "Gets current TX RF bandwidth [Hz].", "", get_tx_rf_bandwidth},
	{"tx_rf_bandwidth=", "Sets the TX RF bandwidth [Hz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_tx_rf_bandwidth},
	{"tx1_attenuation?", "Gets current TX1 attenuation [mdB].", "", get_tx1_attenuation},
	{"tx1_attenuation=", "Sets the TX1 attenuation [mdB] (kept as the operating value).", "", gnss_set_tx1_attenuation},
	{"tx2_attenuation?", "Gets current TX2 attenuation [mdB].", "", get_tx2_attenuation},
	{"tx2_attenuation=", "Sets the TX2 attenuation [mdB].", "", set_tx2_attenuation},
	{"tx_fir_en?", "Gets current TX FIR state.", "", get_tx_fir_en},
	{"tx_fir_en=", "Sets the TX FIR state.", "", set_tx_fir_en},
	{"rx_lo_freq?", "Gets current RX LO frequency [MHz].", "", get_rx_lo_freq},
	{"rx_lo_freq=", "Sets the RX LO frequency [MHz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_rx_lo_freq},
	{"rx_samp_freq?", "Gets current RX sampling frequency [Hz].", "", get_rx_samp_freq},
	{"rx_samp_freq=", "Sets the RX sampling frequency [Hz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_rx_samp_freq},
	{"rx_rf_bandwidth?", "Gets current RX RF bandwidth [Hz].", "", get_rx_rf_bandwidth},
	{"rx_rf_bandwidth=", "Sets the RX RF bandwidth [Hz] (MVDR on: RX+TX, restarts MVDR).", "", gnss_set_rx_rf_bandwidth},
	{"rx1_gc_mode?", "Gets current RX1 GC mode.", "", get_rx1_gc_mode},
	{"rx1_gc_mode=", "Sets the RX1 GC mode (MVDR on: RX1 and RX2 together).", "", gnss_set_rx1_gc_mode},
	{"rx2_gc_mode?", "Gets current RX2 GC mode.", "", get_rx2_gc_mode},
	{"rx2_gc_mode=", "Sets the RX2 GC mode (MVDR on: RX1 and RX2 together).", "", gnss_set_rx2_gc_mode},
	{"rx1_rf_gain?", "Gets current RX1 RF gain.", "", get_rx1_rf_gain},
	{"rx1_rf_gain=", "Sets the RX1 RF gain (MVDR on: RX1 and RX2 together).", "", gnss_set_rx1_rf_gain},
	{"rx2_rf_gain?", "Gets current RX2 RF gain.", "", get_rx2_rf_gain},
	{"rx2_rf_gain=", "Sets the RX2 RF gain (MVDR on: RX1 and RX2 together).", "", gnss_set_rx2_rf_gain},
	{"rx_fir_en?", "Gets current RX FIR state.", "", get_rx_fir_en},
	{"rx_fir_en=", "Sets the RX FIR state.", "", set_rx_fir_en},
	{"dds_tx1_tone1_freq?", "Gets current DDS TX1 Tone 1 frequency [Hz].", "", get_dds_tx1_tone1_freq},
	{"dds_tx1_tone1_freq=", "Sets the DDS TX1 Tone 1 frequency [Hz].", "", set_dds_tx1_tone1_freq},
	{"dds_tx1_tone2_freq?", "Gets current DDS TX1 Tone 2 frequency [Hz].", "", get_dds_tx1_tone2_freq},
	{"dds_tx1_tone2_freq=", "Sets the DDS TX1 Tone 2 frequency [Hz].", "", set_dds_tx1_tone2_freq},
	{"dds_tx1_tone1_phase?", "Gets current DDS TX1 Tone 1 phase [degrees].", "", get_dds_tx1_tone1_phase},
	{"dds_tx1_tone1_phase=", "Sets the DDS TX1 Tone 1 phase [degrees].", "", set_dds_tx1_tone1_phase},
	{"dds_tx1_tone2_phase?", "Gets current DDS TX1 Tone 2 phase [degrees].", "", get_dds_tx1_tone2_phase},
	{"dds_tx1_tone2_phase=", "Sets the DDS TX1 Tone 2 phase [degrees].", "", set_dds_tx1_tone2_phase},
	{"dds_tx1_tone1_scale?", "Gets current DDS TX1 Tone 1 scale.", "", get_dds_tx1_tone1_scale},
	{"dds_tx1_tone1_scale=", "Sets the DDS TX1 Tone 1 scale.", "", set_dds_tx1_tone1_scale},
	{"dds_tx1_tone2_scale?", "Gets current DDS TX1 Tone 2 scale.", "", get_dds_tx1_tone2_scale},
	{"dds_tx1_tone2_scale=", "Sets the DDS TX1 Tone 2 scale.", "", set_dds_tx1_tone2_scale},
	{"dds_tx2_tone1_freq?", "Gets current DDS TX2 Tone 1 frequency [Hz].", "", get_dds_tx2_tone1_freq},
	{"dds_tx2_tone1_freq=", "Sets the DDS TX2 Tone 1 frequency [Hz].", "", set_dds_tx2_tone1_freq},
	{"dds_tx2_tone2_freq?", "Gets current DDS TX2 Tone 2 frequency [Hz].", "", get_dds_tx2_tone2_freq},
	{"dds_tx2_tone2_freq=", "Sets the DDS TX2 Tone 2 frequency [Hz].", "", set_dds_tx2_tone2_freq},
	{"dds_tx2_tone1_phase?", "Gets current DDS TX2 Tone 1 phase [degrees].", "", get_dds_tx2_tone1_phase},
	{"dds_tx2_tone1_phase=", "Sets the DDS TX2 Tone 1 phase [degrees].", "", set_dds_tx2_tone1_phase},
	{"dds_tx2_tone2_phase?", "Gets current DDS TX2 Tone 2 phase [degrees].", "", get_dds_tx2_tone2_phase},
	{"dds_tx2_tone2_phase=", "Sets the DDS TX2 Tone 2 phase [degrees].", "", set_dds_tx2_tone2_phase},
	{"dds_tx2_tone1_scale?", "Gets current DDS TX2 Tone 1 scale.", "", get_dds_tx2_tone1_scale},
	{"dds_tx2_tone1_scale=", "Sets the DDS TX2 Tone 1 scale.", "", set_dds_tx2_tone1_scale},
	{"dds_tx2_tone2_scale?", "Gets current DDS TX2 Tone 2 scale.", "", dds_tx2_tone2_scale},
	{"dds_tx2_tone2_scale=", "Sets the DDS TX2 Tone 2 scale.", "", set_dds_tx2_tone2_scale},
	{"gnss_status_full?", "Prints AD9361 and gnss_passthrough runtime state.", "", get_gnss_status},
	{"gnss_ddr_tx?", "Gets the DDR round-trip replay state.", "", get_gnss_ddr_tx},
	{"gnss_ddr_tx=", "1 = capture RX to DDR and replay it cyclically on TX1, 0 = stop.", "gnss_ddr_tx=0", set_gnss_ddr_tx},
	{"gnss_mvdr?", "Gets the MVDR beamformer state: mode, weights, steering, counters.", "", get_gnss_mvdr},
	{"gnss_mvdr=", "1 = adaptive MVDR nulling on TX1, 0 = raw RX1 (does not stop TX1).", "gnss_mvdr=1", set_gnss_mvdr},
	{"gnss_jam_est?", "Jammer present / no jammer, angle, power, JNR.", "", get_gnss_jam},
	{"gnss_jmon=", "N one-line jammer readings, one per second.", "gnss_jmon=10", set_gnss_jmon},
	{"gnss_jcal=", "1 = calibrate phase with a source at broadside, 0 = clear.", "gnss_jcal=1", set_gnss_jcal},
	{"gnss_jthr=", "Jammer detect threshold, JNR in dB (release 2 dB lower).", "gnss_jthr=6", set_gnss_jthr},
	{"gnss_jinv=", "1 = invert phase sign (angle moves the wrong way), 0 = default.", "gnss_jinv=0", set_gnss_jinv},
	{"gnss_jpcal=", "dBm calibration: known CW level in dBm at the RX input.", "gnss_jpcal=-60", set_gnss_jpcal},
	{"gnss_rxgain?", "RX1/RX2 gain, noise floor vs target, ADC clipping/headroom.", "", get_gnss_rxgain},
	{"gnss_rxgain=", "Manual RX1+RX2 gain in dB, same on both channels.", "gnss_rxgain=30", set_gnss_rxgain},
	{"gnss_ogain?", "MVDR output gain, output clipping, DAC level.", "", get_gnss_ogain},
	{"gnss_ogain=", "MVDR output gain 2^n, n = 0..7 (+6 dB per step), AOLC off.", "gnss_ogain=2", set_gnss_ogain},
	{"gnss_aolc?", "Automatic output level: state, measured output level, gain.", "", get_gnss_aolc},
	{"gnss_aolc=", "Automatic output level: target in dB below FS (6..40), 0 = off.", "gnss_aolc=24", set_gnss_aolc},
	{"gnss_iq?", "Receiver IQ-image correction: coefficients (dBc) and updates.", "", get_gnss_iq},
	{"gnss_iq=", "IQ-image correction: 1 = on, 0 = off, 2 = clear and restart.", "gnss_iq=1", set_gnss_iq},
	{"gnss_coh=", "Jammer detect: coherence > 2^-N (1..15, default 4). Splitter benches.", "gnss_coh=4", set_gnss_coh},
	{"gnss_auto=", "1 = redo the whole automatic MVDR start (as at boot), 0 = TX1 silent.", "gnss_auto=1", set_gnss_auto},
	{"gnss_health?", "Full PASS/WARN/FAIL health check. Changes nothing.", "", get_gnss_health},
	{"gnss_rxauto=", "1 = level the RX gain (noise -60 dBFS, or 10 dB over the chip floor).", "gnss_rxauto=1", set_gnss_rxauto},
	{"gnss_guard?", "ADC clip guard: state, operating / current RX gain, steps.", "", get_gnss_guard},
	{"gnss_guard=", "1 = RX gain steps down by itself when a jammer clips the ADC, 0 = off.", "gnss_guard=1", set_gnss_guard},
	{"gnss_lo?", "LO in use (Hz) and the AD9361 read-back.", "", get_gnss_lo},
	{"gnss_lo=", "LO in MHz, e.g. 1575.42 or 1584.5: re-applies everything (gnss_auto=1).", "gnss_lo=1584.5", set_gnss_lo},
	{"gnss_band?", "List the radio presets (LO, bandwidth, sample rate) and the one in use.", "", get_gnss_band},
	{"gnss_band=", "Select preset 0..5 and restart MVDR with it (4/5 = for barrage jammers).", "gnss_band=4", set_gnss_band},
	{"gnss_plan=", "Custom plan: LO MHz, BW MHz, rate MSPS[, RX gain dB[, TX1 atten mdB]].", "gnss_plan=1575.42,40,45", set_gnss_plan},
	{"gnss_agc?", "RX gain mode: AUTO (MVDR-safe) / MANUAL, gain, target, headroom.", "", get_gnss_agc},
	{"gnss_agc=", "1 = AUTO RX gain (best C/N0, jammer-safe), 0 = MANUAL (keep the gain).", "gnss_agc=1", set_gnss_agc},
	{"gnss_null?", "Null core: 0 = MVDR, 1 = plain PI, 2 = STAP (NLMS), 3 = SMI-MVDR STAP.", "", get_gnss_null},
	{"gnss_null=", "0 = MVDR, 1 = plain PI, 2 = STAP NLMS, 3 = SMI-MVDR STAP (default, best null). No restart.", "gnss_null=3", set_gnss_null},
	{"gnss_supp?", "Jammer suppression: print figure (0..72 dB scale), JNR at the ADC, ACTUAL cancellation.", "", get_gnss_supp},
	{"gnss_stap?", "STAP state: active, step mu, weight norm (noise cost), anti-windup count.", "", get_gnss_stap},
	{"gnss_stap=", "STAP step 2^-mu, 5..10 (default 5 = fastest stable; 6..7 = smoother). Applied at once.", "gnss_stap=5", set_gnss_stap},
	{"gnss_smi?", "SMI-MVDR STAP: window, loading, noise floor, weight norm, solves, constraint.", "", get_gnss_smi},
	{"gnss_smi=", "SMI window 2^k (6..12, def 8 at 16 taps, 9 at 8) [,loading alpha (0.5..1000, def 3)[,DC removal 1/0]].", "gnss_smi=8,3,1", set_gnss_smi},
	{"gnss_steer?", "SMI constraint: power inversion (default) or steered direction.", "", get_gnss_steer},
	{"gnss_steer=", "Steer SMI: theta (deg from broadside, as gnss_jam?) | az,el,bearing of +90 end | 999 = off.", "gnss_steer=30", set_gnss_steer},
	{"gnss_lookph=", "Steer SMI to a raw RX2-RX1 phase in degrees (expert; gnss_steer=999 = off).", "gnss_lookph=0", set_gnss_lookph},
	{"gnss_check?", "ONE-PAGE CHECK (colour): core, image taps, DC removal, loading, suppression, output, verdict.", "", get_gnss_check},
	{"gnss_cmds?", "The anti-jam test commands, in the order you use them.", "", get_gnss_cmds},
	{"gnss_report?", "FULL ANTI-JAM REPORT (copy it): radio, signals, gain, jammer, suppression, weights, events.", "", get_gnss_report},
	{"gnss_floor?", "RX noise vs gain on both channels (output muted ~1 s): finds the AD9361/board floor.", "", get_gnss_floor},
	{"gnss_kpi_full?", "All anti-jam figures on ONE key=value line (for test scripts; na = unknown).", "", get_gnss_kpi},
	{"gnss_spec?", "RX noise spectrum per antenna at the current gain (output muted briefly): flat, edge-rising or spurs.", "", get_gnss_spec},
	{"gnss_spec=", "RX noise spectrum at gain G dB (e.g. gnss_spec=0 for the board floor); gain restored after.", "", set_gnss_spec},
	{"gnss_bist?", "AD9361 digital BIST tone test of the AD9361->FPGA data link (output muted ~0.5 s).", "", get_gnss_bist},
	{"gnss_clk?", "AD9361 RX/TX clock chain now: ADC clock, oversampling, half-band decimation, RX FIR.", "", get_gnss_clk},
	{"gnss_osr=", "ADC oversampling for a test: 0 driver default, 4, 8 or 12 x the sample rate (restarts MVDR).", "gnss_osr=8", set_gnss_osr},
	{"gnss_floor=", "Gain levelled on the floor of: 0 whole band (default, best C/N0), 1 L1/E1/B1C, 2 every signal (restarts MVDR).", "gnss_floor=0", set_gnss_floor},
	{"gnss_smiavg=", "SMI covariance averaged over windows: 0 off, 1..4 = beta 1/2..1/16 (default 2): steadier satellites.", "gnss_smiavg=2", set_gnss_smiavg},
	{"gnss_beam?", "Automatic steering state (gnss_mvdr_auto.c).", "", get_gnss_beam},
	{"gnss_beam=", "Automatic steering: 0 off, 1 auto (no jammer: omni), 2 auto (no jammer: broadside beam).", "gnss_beam=1", set_gnss_beam},
	{"gnss_mvsteer?", "Single-weight MVDR steering vector in use (gnss_null=0).", "", get_gnss_mvsteer},
	{"gnss_mvsteer=", "Single-weight MVDR: steer to theta deg from broadside (999 = power inversion).", "gnss_mvsteer=20", set_gnss_mvsteer},
	{"gnss_color=", "1 = coloured console output (default), 0 = plain text.", "gnss_color=1", set_gnss_color},
};
const unsigned char cmd_no = (sizeof(cmd_list) / sizeof(command));
const unsigned char cmd_ant_end = CMD_ANT_END;

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
extern struct dds_state dds_st;
extern struct ad9361_rf_phy *ad9361_phy;

/* GNSS-CRPA MOD-6: tracks whether the console has enabled retransmission.
 * Mirrors what was commanded; gnss_status? reports what the HARDWARE says. */
static int gnss_tx_on = 0;

/* GNSS-CRPA MOD-11: tracks whether the console has enabled the MVDR path.
 * Mirrors what was commanded; gnss_mvdr? reports what the HARDWARE says. */
static int gnss_mvdr_on = 0;

/* GNSS-CRPA MOD-11/13/15: ONE radio plan for RX1, RX2 and TX1, used by every
 * mode (gnss_tx=1, gnss_mvdr=1, gnss_rxgain=, and the automatic boot start).
 * MOD-15: the values now live in gnss_config.h (LO 1575.42 MHz, 40 MHz RF
 * bandwidth, 45 MSPS, l_clk 180 MHz) -- read the notes there about LO = L1.
 *
 * MANUAL, MATCHED gain is a requirement for MVDR, not a style choice: the
 * covariance assumes RX1 and RX2 keep the same gain for the whole 8192-sample
 * window. Two independent AGCs (gc_mode 2) change each channel's gain on
 * their own, which looks to MVDR like a changing channel. The boot sequence
 * (gnss_autostart.c) levels the gain to a -60 dBFS noise floor (or 10 dB above
 * the AD9361's own floor if that is higher); gnss_rxauto=1 repeats that,
 * gnss_rxgain= sets it by hand. While running, the clip guard (gnss_guard=)
 * lowers the gain by itself when a jammer clips the ADC.
 *
 * The plan is applied ONCE (at boot by the automatic start, otherwise by the
 * first gnss_tx=1 / gnss_mvdr=1 / gnss_rxgain=). Later gnss_tx=1 /
 * gnss_mvdr=1 do NOT re-apply it, so gains and attenuation you set by hand
 * survive switching modes. gnss_auto=1 re-applies everything. */
#define GNSS_L1_LO_HZ          gnss_lo_hz        /* MOD-16: runtime, gnss_lo= */
#define GNSS_RF_BW_HZ          gnss_bw_hz        /* MOD-17: runtime plan */
#define GNSS_SAMP_RATE_HZ      gnss_fs_hz
#define GNSS_RX_GC_MODE        GNSS_CFG_RX_GC_MODE
#define GNSS_RX_GAIN_DB        gnss_gain0_db
#define GNSS_TX1_ATTEN_MDB     gnss_plan_atten_mdb      /* operating attenuation, applied LAST */
#define GNSS_RX_GAIN_MAX_DB    GNSS_CFG_RX_GAIN_MAX_DB

static int      gnss_radio_defaults_done = 0;
/* MOD-16: the LO in use. Starts at GNSS_CFG_LO_HZ; gnss_lo= changes it at run
 * time (A/B test of 1575.42 vs 1584.5 MHz without a rebuild). */
static uint64_t gnss_lo_hz = GNSS_CFG_LO_HZ;
uint64_t gnss_get_lo_hz(void) { return gnss_lo_hz; }

/* MOD-17: the whole radio plan is runtime, chosen with gnss_band=N (presets
 * below) or gnss_plan=LO,BW,FS[,gain[,atten]]. Boot uses GNSS_CFG_BOOT_BAND. */
typedef struct {
	const char *name;
	uint64_t lo_hz;
	uint32_t bw_hz, fs_hz;
	int32_t  gain_db;          /* starting RX gain; the auto gain takes over */
	uint32_t atten_mdb;        /* TX1 operating attenuation */
} gnss_band_t;

static const gnss_band_t gnss_bands[] = {
	/* 0: L1 wide            */ { "L1 wide 40 MHz",     1575420000ULL, 40000000U, 45000000U, 50, 34000U },
	/* 1: GPS L1 C/A narrow  */ { "L1 narrow 4 MHz",    1575420000ULL,  4000000U,  4500000U, 30, 34000U },
	/* 2: L1/E1/B1 51 MSPS   */ { "1584.5 / 51 MHz",    1584500000ULL, 51000000U, 51000000U, 50, 34000U },
	/* 3: MOD-26 default: GPS + Galileo + BeiDou B1I/B1C + GLONASS */
	/* MOD-26b: 61.44 MS/s (l_clk 245.76 MHz: needs the timing-closed MVDR 1.14b v5b+) */
	/* 3: wide 56 MHz        */ { "ALL systems 1582.5 / 56 MHz (default)", 1582500000ULL, 56000000U, 61440000U, 50, 34000U },
	/* MOD-23: presets for WIDEBAND (barrage) jammers. A single complex weight
	 * per antenna can null a jammer only as well as RX1 and RX2 match across
	 * the band; the mismatch (antenna SAW/LNA, cables, AD9361 filters at the
	 * band edges) and the array delay grow with bandwidth. Narrower band =
	 * deeper null on a barrage, and less barrage power at the ADC. The LO
	 * sits where there is no satellite signal (no DC / LO-leakage on GNSS). */
	/* 4: B1I + L1/E1, 20 MHz */ { "barrage: B1I+L1/E1 20 MHz", 1568500000ULL, 20000000U, 22500000U, 50, 34000U },
	/* 5: L1/E1 only, 8 MHz   */ { "barrage: L1/E1 8 MHz",      1577920000ULL,  8000000U, 10000000U, 50, 34000U },
};
#define GNSS_NUM_BANDS ((int)(sizeof(gnss_bands) / sizeof(gnss_bands[0])))

static uint32_t gnss_bw_hz          = GNSS_CFG_RF_BW_HZ;
static uint32_t gnss_fs_hz          = GNSS_CFG_SAMP_RATE_HZ;
static int32_t  gnss_gain0_db       = GNSS_CFG_RX_GAIN_START_DB;
static uint32_t gnss_plan_atten_mdb = GNSS_CFG_TX1_ATTEN_MDB;
static int      gnss_band_now       = 0;     /* -1 = custom (gnss_plan=) */
/* MOD-26b: TX1 level the operator chose with tx1_attenuation= (0 = none):
 * kept through gnss_band= / gnss_auto=1 restarts instead of the preset */
static uint32_t gnss_user_tx1_mdb   = 0;

void gnss_get_plan(uint64_t *lo, uint32_t *bw, uint32_t *fs, int32_t *gain0, uint32_t *atten)
{
	if(lo)    *lo    = gnss_lo_hz;
	if(bw)    *bw    = gnss_bw_hz;
	if(fs)    *fs    = gnss_fs_hz;
	if(gain0) *gain0 = gnss_gain0_db;
	if(atten) *atten = gnss_plan_atten_mdb;
}

/* Selects a preset as the plan (does not touch the radio). */
int gnss_select_band(int n)
{
	if(n < 0 || n >= GNSS_NUM_BANDS)
		return -1;
	gnss_lo_hz          = gnss_bands[n].lo_hz;
	gnss_bw_hz          = gnss_bands[n].bw_hz;
	gnss_fs_hz          = gnss_bands[n].fs_hz;
	gnss_gain0_db       = gnss_bands[n].gain_db;
	gnss_plan_atten_mdb = gnss_user_tx1_mdb ? gnss_user_tx1_mdb : gnss_bands[n].atten_mdb;
	gnss_band_now       = n;
	return 0;
}
/* MOD-26: the preset in use (-1 = custom) and its name, for gnss_report? */
void gnss_band_info(int *n, const char **name)
{
	if(n)    *n    = gnss_band_now;
	if(name) *name = (gnss_band_now >= 0) ? gnss_bands[gnss_band_now].name : "custom (gnss_plan=)";
}

/* Operating TX1 attenuation: what gnss_tx=1 / gnss_mvdr=1 open up to. Tracks
 * a value you set with tx1_attenuation= (captured before each mode change),
 * never the "silent" maximum the abort paths leave behind. */
static uint32_t gnss_op_atten_mdb = GNSS_CFG_TX1_ATTEN_MDB;

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-11/13. Apply the shared RX1/RX2/TX1 radio plan.
 *
 * Reuses the existing set_xxx() console handlers rather than re-deriving the
 * ANTSDR_E310 RF-switch GPIO logic that set_rx_lo_freq()/set_tx_lo_freq()
 * already contain, and they print the normal per-parameter confirmations.
 * LO goes through those MHz-based handlers as Hz/1e6 and back; a double is
 * exact for these integers, so the round trip is bit-exact.
 *
 * Deliberately does NOT touch tx1_attenuation: each TX path opener has its
 * own safety ordering (maximum attenuation before anything opens, the
 * operating value LAST).
 *
 * RX2 must be enabled at the AD9361 2R2T init level for any of this to
 * matter; that cannot be changed at runtime.
*******************************************************************************/
/**************************************************************************//***
 * @brief GNSS-CRPA MOD-30. AD9361 clock chain: read-back and a test override.
 *
 * ad9361_set_rx/tx_sampling_freq() lets the driver pick the chain (its "highest
 * oversampling" rule): ADC clock = sample rate x 12, 8, 6, 4 ... up to 640 MHz,
 * then HB3/HB2/HB1 decimate. gnss_osr=N forces ADC = N x the sample rate with
 * the driver's own divider table and DAC = ADC (<= 320 MHz) or ADC/2, so the
 * ADC noise at the band edges can be compared at another oversampling. The
 * sample rate, l_clk and the LVDS interface do not change. RX FIR bypassed.
*******************************************************************************/
static int gnss_osr = 0;                 /* 0 = driver default */

static int gnss_osr_chain(uint32_t fs, int n, uint32_t *rx, uint32_t *tx)
{
	static const uint8_t dv[7][4] = {
		{12, 3, 2, 2}, {8, 2, 2, 2}, {6, 3, 1, 2}, {4, 2, 2, 1},
		{3, 3, 1, 1},  {2, 2, 1, 1}, {1, 1, 1, 1} };
	uint64_t adc, dac, bb = 0;
	int i, ir = -1, it = -1, d;

	for(i = 0; i < 7; i++)
		if(dv[i][0] == n) ir = i;
	if(ir < 0 || fs == 0)
		return -1;
	adc = (uint64_t)fs * (uint64_t)n;
	if(adc > 640000000ULL || adc < 25000000ULL)
		return -2;                        /* ADC clock outside the AD9361's range */
	dac = (adc > 320000000ULL) ? adc / 2ULL : adc;
	for(i = 0; i < 7; i++)
		if((uint64_t)dv[i][0] * (uint64_t)fs == dac) it = i;
	if(it < 0)
		return -3;
	for(d = 64; d >= 2; d /= 2) {           /* the driver's rule: highest BBPLL <= 1430 MHz */
		bb = adc * (uint64_t)d;
		if(bb <= 1430000000ULL) break;
	}
	if(bb < 715000000ULL || bb > 1430000000ULL)
		return -4;
	rx[0] = (uint32_t)bb;  rx[1] = (uint32_t)adc;
	rx[2] = rx[1] / dv[ir][1];  rx[3] = rx[2] / dv[ir][2];  rx[4] = rx[3] / dv[ir][3];  rx[5] = rx[4];
	tx[0] = (uint32_t)bb;  tx[1] = (uint32_t)dac;
	tx[2] = tx[1] / dv[it][1];  tx[3] = tx[2] / dv[it][2];  tx[4] = tx[3] / dv[it][3];  tx[5] = tx[4];
	return (rx[5] == fs && tx[5] == fs) ? 0 : -5;
}

/* ADC clock / sample rate now (0 = unknown). gnss_kpi? osr= */
int gnss_adc_osr_now(void)
{
	uint32_t rx[6] = {0}, tx[6] = {0};
	if(!ad9361_phy || ad9361_get_trx_path_clks(ad9361_phy, rx, tx) != 0 || rx[5] == 0)
		return 0;
	return (int)((rx[1] + rx[5] / 2U) / rx[5]);
}

static void gnss_mhz2(const char *label, uint32_t hz)
{
	uint32_t c = (hz % 1000000U) / 10000U;          /* console_print has no %02d */
	console_print("%s%d.%d%d", (char *)label, (long)(hz / 1000000U), (long)(c / 10U), (long)(c % 10U));
}

void gnss_clk_print(void)
{
	uint32_t rx[6] = {0}, tx[6] = {0};
	uint8_t  fir = 0;

	if(!ad9361_phy || ad9361_get_trx_path_clks(ad9361_phy, rx, tx) != 0 || rx[5] == 0 || tx[5] == 0) {
		console_print("gnss_clk: clock chain not readable\n");
		return;
	}
	ad9361_get_rx_fir_en_dis(ad9361_phy, &fir);
	console_print("GNSS_CLK: AD9361 clock chain (MHz)\n");
	gnss_mhz2("  RX: BBPLL ", rx[0]); gnss_mhz2("  ADC ", rx[1]); gnss_mhz2("  R2 ", rx[2]);
	gnss_mhz2("  R1 ", rx[3]); gnss_mhz2("  CLKRF ", rx[4]); gnss_mhz2("  sample ", rx[5]); console_print("\n");
	console_print("      ADC = %d x the sample rate; decimation HB3 x%d, HB2 x%d, HB1 x%d, RX FIR x%d (%s)\n",
		      (long)((rx[1] + rx[5] / 2U) / rx[5]), (long)(rx[1] / rx[2]), (long)(rx[2] / rx[3]),
		      (long)(rx[3] / rx[4]), (long)(rx[4] / rx[5]), fir ? "ON" : "bypassed");
	gnss_mhz2("  TX: BBPLL ", tx[0]); gnss_mhz2("  DAC ", tx[1]); gnss_mhz2("  T2 ", tx[2]);
	gnss_mhz2("  T1 ", tx[3]); gnss_mhz2("  CLKTF ", tx[4]); gnss_mhz2("  sample ", tx[5]); console_print("\n");
	console_print("  gnss_osr=%d (%s)\n", (long)gnss_osr,
		      gnss_osr ? "forced for a test; gnss_osr=0 returns to the driver's chain" : "the driver's own chain");
}

/* Called at the end of gnss_radio_apply_defaults(): forces the chain when
 * gnss_osr != 0. The driver re-tunes the analog filters and the ADC for it. */
void gnss_osr_apply(void)
{
	uint32_t rx[6], tx[6], fs = 0;
	uint8_t  fir = 0;
	int r;

	if(!ad9361_phy || gnss_osr == 0)
		return;
	ad9361_get_rx_sampling_freq(ad9361_phy, &fs);
	ad9361_get_rx_fir_en_dis(ad9361_phy, &fir);
	if(fir) {
		console_print("gnss_osr: RX FIR is ON (rx_fir_en=0 first); driver's chain kept\n");
		return;
	}
	r = gnss_osr_chain(fs, gnss_osr, rx, tx);
	if(r != 0) {
		console_print("gnss_osr: ADC = %d x %d SPS is not possible on the AD9361 (code %d); "
			      "driver's chain kept\n", (long)gnss_osr, (long)fs, (long)r);
		return;
	}
	if(ad9361_set_trx_path_clks(ad9361_phy, rx, tx) != 0) {
		console_print("gnss_osr: the driver refused the chain; back to its own\n");
		ad9361_set_rx_sampling_freq(ad9361_phy, fs);
		ad9361_set_tx_sampling_freq(ad9361_phy, fs);
		return;
	}
	console_print("gnss_osr: ADC clock %d x the sample rate (forced)\n", (long)gnss_osr);
}

void get_gnss_clk(double* param, char param_no) { gnss_clk_print(); }

void set_gnss_osr(double* param, char param_no)
{
	int n;
	if(param_no < 1) {
		console_print("gnss_osr= needs 0 (driver default), 4, 8 or 12\n");
		return;
	}
	n = (int)(param[0] + 0.5);
	if(n != 0 && n != 4 && n != 8 && n != 12) {
		console_print("gnss_osr: 0, 4, 8 or 12 only\n");
		return;
	}
	gnss_osr = n;
	console_print("GNSS_OSR: %d -- restarting MVDR with it; then gnss_clk? and gnss_spec=0\n", (long)n);
	(void)gnss_autostart_run();
}

/* MOD-31: covariance averaging of the SMI solver (no restart needed) */
void set_gnss_smiavg(double* param, char param_no)
{
	int k;
	if(param_no < 1 || param[0] < 0.0 || param[0] > 4.0) {
		console_print("gnss_smiavg= 0 off (one window per weight set), 1..4 = beta 1/2, 1/4 (default), 1/8, 1/16\n");
		return;
	}
	k = (int)(param[0] + 0.5);
	gnss_smi_set_avg(k);
	if(k)
		console_print("GNSS_SMIAVG: covariance averaged with beta 1/%d (~%d windows), restarted on any power step\n",
			      (long)(1 << k), (long)(1 << k));
	else
		console_print("GNSS_SMIAVG: off, every window solved on its own (MOD-30)\n");
}

void set_gnss_floor(double* param, char param_no)
{
	int m;
	if(param_no < 1 || param[0] < 0.0 || param[0] > 2.0) {
		console_print("gnss_floor= 0 whole band (default, highest gain), 1 GPS L1/Galileo E1/B1C, 2 every signal in the band\n");
		return;
	}
	m = (int)(param[0] + 0.5);
	gnss_floor_mode_set(m);
	console_print("GNSS_FLOOR: gain levelled on the floor of: %s -- restarting MVDR\n",
		      (char *)gnss_floor_mode_name(m));
	(void)gnss_autostart_run();
}

void gnss_radio_apply_defaults(void)
{
	double p[1];

	if(!ad9361_phy) {
		console_print("gnss radio: ad9361_phy is NULL; defaults not applied\n");
		return;
	}
	/* MOD-15: sample rate BEFORE bandwidth. The AD9361 re-tunes its analog
	 * filters from the stored bandwidth whenever the clock chain changes, so
	 * the bandwidth is written last, once the new rate is in. */
	p[0] = (double)GNSS_L1_LO_HZ / 1000000.0;   set_rx_lo_freq(p, 1);
	p[0] = GNSS_SAMP_RATE_HZ;                   set_rx_samp_freq(p, 1);
	p[0] = GNSS_RF_BW_HZ;                       set_rx_rf_bandwidth(p, 1);
	p[0] = GNSS_RX_GC_MODE;                     set_rx1_gc_mode(p, 1);
	p[0] = GNSS_RX_GAIN_DB;                     set_rx1_rf_gain(p, 1);
	p[0] = GNSS_RX_GC_MODE;                     set_rx2_gc_mode(p, 1);
	p[0] = GNSS_RX_GAIN_DB;                     set_rx2_rf_gain(p, 1);

	p[0] = (double)GNSS_L1_LO_HZ / 1000000.0;   set_tx_lo_freq(p, 1);
	p[0] = GNSS_SAMP_RATE_HZ;                   set_tx_samp_freq(p, 1);
	p[0] = GNSS_RF_BW_HZ;                       set_tx_rf_bandwidth(p, 1);
	gnss_osr_apply();                    /* MOD-30: gnss_osr= (0 = driver's clock chain) */

	gnss_op_atten_mdb        = GNSS_TX1_ATTEN_MDB;
	gnss_radio_defaults_done = 1;
	(void)gnss_tx2_hold();               /* MOD-26: TX2 unused, always quiet */
	gnss_pt_set_fs(GNSS_SAMP_RATE_HZ);   /* MOD-17: rate-dependent MVDR timings */
	console_print("gnss radio: defaults applied - LO %d Hz, BW %d Hz, %d SPS, "
		      "RX1/RX2 manual %d dB, TX1 operating attenuation %d mdB\n",
		      (long)GNSS_L1_LO_HZ, (long)GNSS_RF_BW_HZ, (long)GNSS_SAMP_RATE_HZ,
		      (long)GNSS_RX_GAIN_DB, (long)GNSS_TX1_ATTEN_MDB);
}

/* MOD-19: the automatic start silences TX1 while it sets up, without the
 * "GNSS_TX: OFF" message (that read as if the board would stay silent), and
 * records the result when it opens TX1 again. */
void gnss_tx_silence_quiet(void)
{
	if(!ad9361_phy)
		return;
	gnss_l1_set_dac_source(ad9361_phy, AXI_DAC_DATA_SEL_ZERO);
	gnss_pt_set_mute(1);
	gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
	gnss_tx_on = 0;
}

void gnss_tx_mark_on(int mvdr)
{
	gnss_tx_on   = 1;
	gnss_mvdr_on = mvdr ? 1 : 0;
}
int gnss_tx_is_on(void) { return gnss_tx_on; }      /* MOD-26 */

/* MOD-26: TX2 is not used (TX1 carries the GNSS output to the receiver):
 * hold it at GNSS_CFG_TX2_ATTEN_MDB. Returns the read-back value. */
uint32_t gnss_tx2_hold(void)
{
	uint32_t a = 0;
	if(!ad9361_phy)
		return 0;
	if(ad9361_get_tx_attenuation(ad9361_phy, 1, &a) != 0 || a != GNSS_CFG_TX2_ATTEN_MDB)
		ad9361_set_tx_attenuation(ad9361_phy, 1, GNSS_CFG_TX2_ATTEN_MDB);
	ad9361_get_tx_attenuation(ad9361_phy, 1, &a);
	return a;
}

/* MOD-15: used by gnss_autostart.c. The operating attenuation is what
 * gnss_tx=1 / gnss_mvdr=1 open TX1 up to, applied as the very last step. */
uint32_t gnss_get_op_atten_mdb(void)        { return gnss_op_atten_mdb; }
void     gnss_set_op_atten_mdb(uint32_t mdb) { gnss_op_atten_mdb = mdb; }

static void gnss_radio_ensure_defaults(void)
{
	if(!gnss_radio_defaults_done)
		gnss_radio_apply_defaults();
}

/* Remember the operating attenuation before a mode change drives it to the
 * maximum. Skipped until the defaults are in (the boot value is not ours)
 * and when it is already at the maximum (a previous abort, not a choice). */
static void gnss_capture_op_atten(void)
{
	uint32_t a = 0;

	if(!gnss_radio_defaults_done || !ad9361_phy)
		return;
	if(ad9361_get_tx_attenuation(ad9361_phy, 0, &a) == 0 &&
	   a < (uint32_t)GNSS_L1_TX_MAX_ATTEN_MDB)
		gnss_op_atten_mdb = a;
}

/* MVDR needs manual, equal gain on RX1 and RX2. Warn (do not override: the
 * user may be experimenting on purpose). Returns 1 if OK. */
static int gnss_check_matched_gain(void)
{
	uint8_t  m1 = 0xFF, m2 = 0xFF;
	int32_t  g1 = 0, g2 = 0;

	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	if(m1 != 0 || m2 != 0 || g1 != g2) {
		console_print("  WARNING: MVDR needs MANUAL and EQUAL gain on RX1/RX2. Now: "
			      "RX1 mode %d gain %d dB, RX2 mode %d gain %d dB.\n"
			      "           Fix with gnss_rxgain=%d\n",
			      (long)m1, (long)g1, (long)m2, (long)g2, (long)g1);
		return 0;
	}
	return 1;
}

/* Forward declaration: defined further down, next to get_gnss_mvdr() which is
 * its main caller, but set_gnss_mvdr() below also uses it to prove the radio
 * plan actually landed, read back live, right when it finishes enabling. */
static void print_mvdr_txrx_parameters(void);

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-6. Enable or disable the RX1 -> PL -> TX1 retransmit.
 *
 * SAFETY ORDERING. Attenuation is driven to the hardware maximum (89.75 dB,
 * about -83 dBm out) BEFORE any sample path is opened, and again after
 * gnss_l1_enable_tx(), because that function deliberately leaves attenuation at
 * its 30 dB "safe start" -- about -23 dBm, which is 60 dB hotter than we want
 * for a first conducted test into a GNSS module. The operating attenuation
 * (GNSS_CFG_TX1_ATTEN_MDB in gnss_config.h, 64000 mdB) is applied LAST, once passthrough, mute and the DAC
 * source are all confirmed -- so nothing gets louder than silent until every
 * earlier step has succeeded. Step further with tx1_attenuation= (SMALLER
 * number = LOUDER). gnss_tx=0 silences the transmitter in one command and is
 * the abort path.
 *
 * GNSS-CRPA MOD-11/13: makes sure the shared RX1/RX2/TX1 radio plan is in
 * (gnss_radio_ensure_defaults(): applied once, never re-applied), so a
 * conducted test looks the same whether or not MVDR is in the loop. The
 * final attenuation is the remembered operating value (GNSS_CFG_TX1_ATTEN_MDB,
 * or whatever tx1_attenuation= last set), not a fixed number.
 *
 * This is a procedural interlock, not an RF safety device. It cannot detect
 * what is physically attached to TX1.
*******************************************************************************/
void gnss_tx_raw(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_tx= needs 0 or 1\n");
		return;
	}
	if(!ad9361_phy) {
		console_print("gnss_tx: ad9361_phy is NULL; cannot touch the radio\n");
		return;
	}

	if((int)param[0] != 0) {
		gnss_capture_op_atten();
		/* Quietest possible BEFORE the data path opens. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_tx: attenuation not confirmed; NOT transmitting\n");
			return;
		}
		/* GNSS-CRPA MOD-13: the one shared radio plan, applied once. */
		gnss_radio_ensure_defaults();
		if(gnss_l1_enable_tx(ad9361_phy,
				GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION) != GNSS_L1_OK) {
			return;
		}
		/* enable_tx() just set 30 dB. Put it back to the maximum. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_tx: attenuation not confirmed; NOT transmitting\n");
			return;
		}

		gnss_pt_set_passthrough(1);
		gnss_pt_set_mute(0);
		if(gnss_l1_set_dac_source(ad9361_phy,
				AXI_DAC_DATA_SEL_DMA) != GNSS_L1_OK) {
			/* The DAC did not accept our samples. Close everything again
			 * rather than leaving the path half open. */
			gnss_pt_set_mute(1);
			gnss_pt_set_passthrough(0);
			console_print("gnss_tx: DAC source NOT confirmed; TX left silent\n");
			return;
		}

		/* Requested operating attenuation, applied last of all -- same
		 * pattern as set_gnss_mvdr(): nothing gets louder than silent until
		 * every earlier step above has succeeded. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy, gnss_op_atten_mdb) != GNSS_L1_OK) {
			console_print("gnss_tx: WARNING final attenuation not confirmed; "
				      "check tx1_attenuation?\n");
		}

		gnss_tx_on = 1;
		console_print("GNSS_TX: ON  - %s is being transmitted on TX1 at "
			      "%d mdB attenuation.\n",
			      (gnss_pt_read(GNSS_PT_REG_STATUS) & GNSS_PT_ST_MVDR_SEL_SYNCED)
			          ? "the MVDR output (jammer nulled)" : "raw RX1 (no nulling)",
			      (long)gnss_op_atten_mdb);
		console_print("         Raise/lower power with tx1_attenuation= "
			      "(smaller = louder). Abort with gnss_tx=0\n");
	} else {
		gnss_capture_op_atten();
		gnss_l1_set_dac_source(ad9361_phy, AXI_DAC_DATA_SEL_ZERO);
		gnss_pt_set_mute(1);
		gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
		gnss_tx_on = 0;
		console_print("GNSS_TX: OFF - DAC source ZERO, passthrough muted, "
			      "attenuation at maximum.\n");
	}
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-6. Report the retransmit state, read back from hardware.
*******************************************************************************/
void gnss_tx_hw_print(double* param, char param_no)
{
	uint32_t status, atten = 0;
	int dac_i, dac_q, pass_en, transmitting;

	if(!ad9361_phy) {
		console_print("gnss_tx: ad9361_phy is NULL\n");
		return;
	}
	status = gnss_pt_read(GNSS_PT_REG_STATUS);
	dac_i   = (status & GNSS_PT_ST_DAC_EN_I0) ? 1 : 0;
	dac_q   = (status & GNSS_PT_ST_DAC_EN_Q0) ? 1 : 0;
	pass_en = (status & GNSS_PT_ST_PASS_EN_SYNCED) ? 1 : 0;
	ad9361_get_tx_attenuation(ad9361_phy, 0, &atten);

	/* Lead with what the HARDWARE says, not with what was commanded.
	 *
	 * dac_enable is axi_ad9361's read-back of (dac_data_sel == 4'h2) and
	 * pass_en comes back through STATUS, so together they say whether samples
	 * are actually reaching the DAC from the passthrough.
	 *
	 * `gnss_tx_on` only tracks the gnss_tx= console command. The MOD-8
	 * deployment auto-start does NOT set it, so on an SD-booted board this
	 * printed "commanded OFF" while the board was plainly transmitting --
	 * observed 2026-09-15. A status line that says OFF during transmission is
	 * exactly the class of misleading signal that cost this project three days
	 * on ISSUE-0003, so the derived hardware state now leads. */
	transmitting = (dac_i && dac_q && pass_en) ? 1 : 0;

	/* MOD-19: report ONLY what the hardware does (the old "console flag"
	 * line said OFF while the automatic start was transmitting MVDR). */
	console_print("GNSS_TX: %s - %s on TX1, TX1 attenuation %d mdB%s\n",
		      transmitting ? "ON (TRANSMITTING)" : "OFF (silent)",
		      !transmitting ? "nothing"
		      : ((status & GNSS_PT_ST_MVDR_SEL_SYNCED) ? "MVDR output (jammer nulled)"
		                                                 : "raw RX1 (no nulling)"),
		      (long)atten,
		      (transmitting && atten >= GNSS_L1_TX_MAX_ATTEN_MDB) ? " (maximum = quietest)" : "");
	console_print("         read from hardware: pass_en %d, dac_enable i/q %d / %d\n",
		      (long)pass_en, (long)dac_i, (long)dac_q);
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-6. Dump AD9361 and gnss_passthrough runtime state.
*******************************************************************************/
void get_gnss_status(double* param, char param_no)
{
	if(!ad9361_phy) {
		console_print("gnss_status: ad9361_phy is NULL\n");
		return;
	}
	gnss_l1_print_status(ad9361_phy);
	gnss_pt_print_state();
	{
		uint32_t rate = 0;
		ad9361_get_rx_sampling_freq(ad9361_phy, &rate);
		gnss_pt_print_lclk(rate);
	}
}

/* GNSS-CRPA MOD-7: the DDR round trip.
 * rx_dmac / tx_dmac are created unconditionally in main.c (~line 715). */
extern struct axi_dmac *rx_dmac;
extern struct axi_dmac *tx_dmac;

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-9/MOD-10. The main menu, bound to "?".
 *
 * Delegates to gnss_info_menu() so that "?" and the start-up banner print the
 * SAME thing. Two menus that could drift apart is how a console starts lying.
*******************************************************************************/
void get_menu(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_ant_menu();		/* LVDS / ant board: the new main menu */
}

/* LVDS / ant board: gnss_ant.c asks whether the plan is already in */
int gnss_radio_defaults_ready(void) { return gnss_radio_defaults_done; }
/**************************************************************************//***
 * @brief GNSS-CRPA MOD-7. Start or stop the DDR round-trip replay.
 *
 * RX -> DMA -> DDR -> DMA -> axi_ad9361 -> TX1, i.e. the branch of the
 * gnss_passthrough mux selected by pass_en = 0.
 *
 * Same safety ordering as gnss_tx=: attenuation to the hardware maximum BEFORE
 * any sample path opens, and gnss_txdma_start() switches the DAC data source
 * LAST, so no RF leaves TX1 until everything else is confirmed.
 *
 * Mutually exclusive with gnss_tx= (the live passthrough), which drives the
 * other branch of the same mux. Starting one stops the other.
*******************************************************************************/
void set_gnss_ddr_tx(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_ddr_tx= needs 0 or 1\n");
		return;
	}
	if(!ad9361_phy) {
		console_print("gnss_ddr_tx: ad9361_phy is NULL\n");
		return;
	}
	if(!rx_dmac || !tx_dmac) {
		console_print("gnss_ddr_tx: rx_dmac or tx_dmac is NULL; the DMA cores "
			      "were never initialised\n");
		return;
	}

	if((int)param[0] != 0) {
		/* Quietest the hardware can manage, BEFORE anything opens. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_ddr_tx: attenuation not confirmed; NOT transmitting\n");
			return;
		}
		if(gnss_l1_enable_tx(ad9361_phy,
				GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION) != GNSS_L1_OK) {
			return;
		}
		/* enable_tx() just set its own 30 dB default. Back to maximum. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_ddr_tx: attenuation not confirmed; NOT transmitting\n");
			return;
		}

		if(gnss_txdma_start(ad9361_phy, rx_dmac, tx_dmac) != GNSS_TXDMA_OK) {
			return;
		}
		console_print("GNSS_DDR_TX: ON  - replaying DDR on TX1 at %d mdB "
			      "attenuation (quietest).\n",
			      (long)GNSS_L1_TX_MAX_ATTEN_MDB);
		console_print("             Raise power with tx1_attenuation= "
			      "(smaller = louder). Abort with gnss_ddr_tx=0\n");
	} else {
		(void)gnss_txdma_stop(ad9361_phy, tx_dmac);
		gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
		console_print("GNSS_DDR_TX: OFF - replay halted, DAC source ZERO, "
			      "attenuation at maximum.\n");
	}
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-7. Report the DDR round-trip state, read back from
 *        hardware where possible.
*******************************************************************************/
void get_gnss_ddr_tx(double* param, char param_no)
{
	uint32_t status, atten = 0;
	int dac_i, dac_q, pass_en;

	if(!ad9361_phy) {
		console_print("gnss_ddr_tx: ad9361_phy is NULL\n");
		return;
	}
	status  = gnss_pt_read(GNSS_PT_REG_STATUS);
	dac_i   = (status & GNSS_PT_ST_DAC_EN_I0) ? 1 : 0;
	dac_q   = (status & GNSS_PT_ST_DAC_EN_Q0) ? 1 : 0;
	pass_en = (status & GNSS_PT_ST_PASS_EN_SYNCED) ? 1 : 0;
	ad9361_get_tx_attenuation(ad9361_phy, 0, &atten);

	console_print("GNSS_DDR_TX: replay %s, pass_en = %d (%s branch), "
		      "dac_enable i/q = %d / %d, TX1 attenuation %d mdB\n",
		      gnss_txdma_is_running() ? "RUNNING" : "stopped",
		      (long)pass_en, pass_en ? "live passthrough" : "DMA/DDR",
		      (long)dac_i, (long)dac_q, (long)atten);
	if(gnss_txdma_is_running() && !(dac_i && dac_q)) {
		console_print("             WARNING: replay running but the DAC is NOT "
			      "taking our samples.\n");
	}
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-11. Enable or disable adaptive 2-channel MVDR nulling
 *        on TX1.
 *
 * SAFETY ORDERING, same interlock as set_gnss_tx(): attenuation is driven to
 * the hardware maximum BEFORE any sample path opens, and again after
 * gnss_l1_enable_tx() (which leaves its own 30 dB default). The operating
 * attenuation (GNSS_CFG_TX1_ATTEN_MDB, or the last tx1_attenuation= value) is
 * applied LAST, once passthrough, mute, DAC source and mvdr_sel are all
 * confirmed -- so nothing gets louder than "silent" until every earlier
 * step has succeeded.
 *
 * GNSS-CRPA MOD-13: does NOT re-apply the radio plan if it is already in, so
 * the RX gain set with gnss_rxgain= survives gnss_mvdr=1. Does not touch
 * OUT_GAIN either (it keeps whatever gnss_aolc= / gnss_ogain= last set; the
 * automatic start turns AOLC on).
 *
 * gnss_mvdr=0 only clears CONTROL.mvdr_sel and sets the MVDR core to bypass
 * mode: it does NOT touch pass_en, mute, DAC source or attenuation. If
 * gnss_tx=1 is running, TX1 keeps transmitting raw RX1 afterwards -- turning
 * MVDR off is not an abort path. Use gnss_tx=0 for that.
 *
 * PREREQUISITE THIS CANNOT FIX AT RUNTIME: MVDR needs RX2 sampling, which
 * needs the AD9361 profile to be 2R2T. That is set when the radio is brought
 * up (ad9361_init/default_init_param), not by any register this function
 * writes. If RX2 was never enabled that way, gnss_mvdr=1 will report success
 * (every step below is a register write, and register writes succeed) but
 * N_BLOCKS in gnss_mvdr? will stay at 0 forever -- see its VERDICT text.
*******************************************************************************/
void set_gnss_mvdr(double* param, char param_no)
{
	uint32_t v, id;

	if(param_no < 1) {
		console_print("gnss_mvdr= needs 0 or 1\n");
		return;
	}
	if(!ad9361_phy) {
		console_print("gnss_mvdr: ad9361_phy is NULL; cannot touch the radio\n");
		return;
	}

	if((int)param[0] != 0) {
		id = gnss_pt_read(GNSS_PT_REG_MVDR_ID);
		if(id != GNSS_PT_MVDR_EXPECTED_ID) {
			console_print("gnss_mvdr: MVDR core not found (id=0x%08x, expected "
				      "0x%08x). This bitstream does not have it -- reprogram "
				      "the FPGA.\n",
				      (long)id, (long)GNSS_PT_MVDR_EXPECTED_ID);
			return;
		}

		gnss_capture_op_atten();
		/* Quietest possible BEFORE any sample path opens. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_mvdr: attenuation not confirmed; NOT transmitting\n");
			return;
		}

		/* Manual, matched gain on both channels -- required, not a style
		 * choice: MVDR's covariance estimate assumes RX1/RX2 keep the same
		 * gain, which independent AGC (mode 2) cannot guarantee. (Before
		 * MOD-13 this line applied slow AGC at 71 dB -- the opposite of
		 * what this comment always asked for.) */
		gnss_radio_ensure_defaults();
		(void)gnss_check_matched_gain();

		if(gnss_l1_enable_tx(ad9361_phy,
				GNSS_L1_PATH_CONDUCTED_WITH_ATTENUATION) != GNSS_L1_OK) {
			return;
		}
		/* enable_tx() just set its own 30 dB default. Back to maximum, one
		 * more time, before the MVDR/DAC path opens below. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_mvdr: attenuation not confirmed; NOT transmitting\n");
			return;
		}

		/* Known-good MVDR configuration, written every time rather than
		 * trusting whatever was left from an earlier session
		 * (gnss_pt_mvdr_write_defaults(), version-aware):
		 *   A1=[1,0] A2=[0,0]  -- power inversion, needs no calibration
		 *   MVDR >= 1.7: EPS_NF=8 (loading = noise floor), EPS_REL=63 (off).
		 *     MOD-14 FIX: this used to write EPS_REL=10. On a >= 1.7
		 *     bitstream that ADDS loading proportional to the jammer and
		 *     caps the null at ~60 dB JNR -- exactly what 1.7 removed.
		 *   MVDR >= 1.8: hold 0.3 s / glide 0.3 s (COH_SH kept if tuned)
		 *   MVDR >= 1.9: IQ-image correction on
		 *   MVDR >= 1.13 (MOD-24): STAP_CTRL -- gnss_null=2 (default): STAP
		 *     drives the output, mu 2^-gnss_stap, normalisation hold on
		 *   mode=1 (adaptive), mean removal on (CTRL[4]=0), cnt_clear set
		 *     so the counters gnss_mvdr? reports start from zero
		 * Two commits: one with cnt_clear held, one to release it, matching
		 * how gnss_pt_clear_counters() handles the equivalent top-level bit.
		 * OUT_GAIN (manual n or AOLC) is NOT touched: gnss_ogain= / gnss_aolc=. */
		gnss_pt_mvdr_write_defaults();
		gnss_pt_write(GNSS_PT_REG_MVDR_CTRL,
			      GNSS_PT_MVDR_MODE_ADAPTIVE | GNSS_PT_MVDR_CTRL_CNT_CLEAR);
		if(gnss_pt_mvdr_commit() != GNSS_PT_OK) {
			console_print("gnss_mvdr: MVDR config not applied; NOT enabling\n");
			return;
		}
		gnss_pt_write(GNSS_PT_REG_MVDR_CTRL, GNSS_PT_MVDR_MODE_ADAPTIVE);
		if(gnss_pt_mvdr_commit() != GNSS_PT_OK) {
			console_print("gnss_mvdr: cnt_clear release did not commit\n");
			return;
		}
		/* Sliding window (MVDR >= 1.3): 2^6 windows keeps the jammer
		 * estimate at ~1.3-1.5 ms, as validated on v1.2. Older builds: leave. */
		if(gnss_pt_read(GNSS_PT_REG_MVDR_VERSION) >= GNSS_PT_MVDR_VERSION_CLIP)
			(void)gnss_pt_jam_set_avg(gnss_pt_jam_avg_log2());   /* ~1.5 ms at any rate */

		gnss_pt_set_passthrough(1);
		gnss_pt_set_mute(0);
		if(gnss_l1_set_dac_source(ad9361_phy,
				AXI_DAC_DATA_SEL_DMA) != GNSS_L1_OK) {
			gnss_pt_set_mute(1);
			gnss_pt_set_passthrough(0);
			console_print("gnss_mvdr: DAC source NOT confirmed; TX left silent\n");
			return;
		}

		/* Route ch0 to the MVDR output LAST, after everything else above is
		 * confirmed open. */
		v = gnss_pt_read(GNSS_PT_REG_CONTROL);
		gnss_pt_write(GNSS_PT_REG_CONTROL, v | GNSS_PT_CTRL_MVDR_SEL);

		/* Requested operating attenuation, applied last of all. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy, gnss_op_atten_mdb) != GNSS_L1_OK) {
			console_print("gnss_mvdr: WARNING final attenuation not confirmed; "
				      "check tx1_attenuation?\n");
		}

		gnss_mvdr_on = 1;
		console_print("GNSS_MVDR: ON  - adaptive 2-channel nulling on TX1 at "
			      "%d mdB attenuation.\n", (long)gnss_op_atten_mdb);
		console_print("           Input level + output level: gnss_rxauto=1, then "
			      "gnss_aolc=24 (the automatic start\n"
			      "           at boot / gnss_auto=1 does both and opens TX1 last).\n");
		console_print("           Steering = power inversion (no calibration "
			      "needed). Check it with gnss_mvdr?.\n");
		console_print("           Raise/lower power with tx1_attenuation=. "
			      "Abort with gnss_tx=0 (not gnss_mvdr=0).\n");
		/* The per-parameter lines gnss_radio_apply_defaults() printed further up
		 * (from inside set_rx_lo_freq()/set_tx_lo_freq()) echo back in whole
		 * MHz by that command's own long-standing design -- e.g.
		 * "rx_lo_freq=1575" for a 1575419998 Hz request -- which looks like
		 * the fractional part was dropped even though the AD9361 was set
		 * correctly (ad9361_set_rx_lo_freq() is called with the full,
		 * untruncated Hz value; only that confirmation echo divides for
		 * display). Read every parameter back from hardware now, in Hz, so
		 * there is no ambiguity about what actually landed. */
		print_mvdr_txrx_parameters();
	} else {
		v = gnss_pt_read(GNSS_PT_REG_CONTROL);
		gnss_pt_write(GNSS_PT_REG_CONTROL, v & ~GNSS_PT_CTRL_MVDR_SEL);

		gnss_pt_write(GNSS_PT_REG_MVDR_CTRL, GNSS_PT_MVDR_MODE_BYPASS);
		(void)gnss_pt_mvdr_commit();

		gnss_mvdr_on = 0;
		console_print("GNSS_MVDR: OFF - ch0 is back to raw RX1.\n");
		console_print("           This does NOT silence TX1: whatever gnss_tx= "
			      "last set keeps running. Use gnss_tx=0 to actually stop "
			      "transmitting.\n");
	}
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-11. Live RX/TX radio parameters, read straight from
 *        the AD9361 and printed in the same visual format as
 *        gnss_l1_print_status()'s "AD9361 runtime state" block (see
 *        gnss_status?), under its own "MVDR TXRX Parameters" heading.
 *
 * This makes gnss_mvdr? self-contained: what the MVDR core is doing AND
 * what the radio is actually set to, in one command, without also having to
 * run gnss_status?. Always reads back from hardware -- if something else
 * changed a setting since gnss_mvdr=1 ran, this shows that, not what
 * gnss_radio_apply_defaults() last wrote.
*******************************************************************************/
/* Name for an AD9361 gain control mode, as used by rx1_gc_mode= / rx2_gc_mode=. */
static const char *gc_mode_name(uint8_t mode)
{
	switch(mode) {
	case 0:  return "manual";
	case 1:  return "fast attack AGC";
	case 2:  return "slow attack AGC";
	case 3:  return "hybrid AGC";
	default: return "unknown";
	}
}

static void print_mvdr_txrx_parameters(void)
{
	uint64_t rx_lo = 0, tx_lo = 0;
	uint32_t rx_rate = 0, tx_rate = 0, rx_bw = 0, tx_bw = 0, atten = 0;
	int32_t  rx1_gain = 0, rx2_gain = 0;
	uint8_t  rx1_mode = 0xFF, rx2_mode = 0xFF;

	ad9361_get_rx_lo_freq(ad9361_phy, &rx_lo);
	ad9361_get_tx_lo_freq(ad9361_phy, &tx_lo);
	ad9361_get_rx_sampling_freq(ad9361_phy, &rx_rate);
	ad9361_get_tx_sampling_freq(ad9361_phy, &tx_rate);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &rx_bw);
	ad9361_get_tx_rf_bandwidth(ad9361_phy, &tx_bw);
	ad9361_get_tx_attenuation(ad9361_phy, 0, &atten);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &rx1_mode);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &rx2_mode);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &rx1_gain);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &rx2_gain);

	console_print("\n--- MVDR TXRX Parameters ---\n");
	console_print("  RX LO          : %d Hz\n", (long)rx_lo);
	console_print("  TX LO          : %d Hz\n", (long)tx_lo);
	console_print("  RX sample rate : %d Hz\n", (long)rx_rate);
	console_print("  TX sample rate : %d Hz\n", (long)tx_rate);
	console_print("  RX bandwidth   : %d Hz\n", (long)rx_bw);
	console_print("  TX bandwidth   : %d Hz\n", (long)tx_bw);
	console_print("  TX1 attenuation: %d mdB\n", (long)atten);
	console_print("  RX1 gc mode    : %d (%s)\n", (long)rx1_mode, (char *)gc_mode_name(rx1_mode));
	console_print("  RX1 gain       : %d dB\n", (long)rx1_gain);
	console_print("  RX2 gc mode    : %d (%s)\n", (long)rx2_mode, (char *)gc_mode_name(rx2_mode));
	console_print("  RX2 gain       : %d dB\n", (long)rx2_gain);
	if(rx1_mode != 0 || rx2_mode != 0 || rx1_gain != rx2_gain)
		console_print("  WARNING: MVDR needs MANUAL, EQUAL RX1/RX2 gain "
			      "(gnss_rxgain=N sets both).\n");
	/* The MVDR datapath clock, measured: the check for the timing question
	 * (constraint vs real l_clk) that Vivado cannot answer by itself. */
	gnss_pt_print_lclk(rx_rate);
	console_print("-----------------------------------\n");
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-11. Report the MVDR beamformer state, read back from
 *        hardware. Delegates to gnss_pt_mvdr_print_state() for the same
 *        reason get_gnss_status() delegates to gnss_pt_print_state(): one
 *        place owns the register-level detail.
*******************************************************************************/
void get_gnss_mvdr(double* param, char param_no)
{
	if(!ad9361_phy) {
		console_print("gnss_mvdr: ad9361_phy is NULL\n");
		return;
	}
	gnss_pt_mvdr_print_state();
	print_mvdr_txrx_parameters();
	console_print("  gnss_mvdr= console flag: %s\n", gnss_mvdr_on ? "ON" : "OFF");
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-12. Jammer estimator console commands. All the maths
 *        runs in the FPGA (jammer_est.v); these only read, print and
 *        configure. Driver code: gnss_pt_jam_*() in gnss_passthrough.c.
*******************************************************************************/
/* Reads the live RX1/RX2 RF gain, needed to turn dBFS into dB / dBm. */
static int gnss_jam_read_gain(int32_t *g1, int32_t *g2)
{
	*g1 = 0;
	*g2 = 0;
	if(!ad9361_phy)
		return 0;
	if(ad9361_get_rx_rf_gain(ad9361_phy, 0, g1) != 0)
		return 0;
	if(ad9361_get_rx_rf_gain(ad9361_phy, 1, g2) != 0)
		return 0;
	return 1;
}

void get_gnss_jam(double* param, char param_no)
{
	int32_t g1, g2;
	int ok = gnss_jam_read_gain(&g1, &g2);

	gnss_pt_jam_print_state(g1, g2, ok);
}

void set_gnss_jmon(double* param, char param_no)
{
	long n;

	if(param_no < 1) {
		console_print("gnss_jmon= needs a count, e.g. gnss_jmon=10\n");
		return;
	}
	n = (long)param[0];
	if(n < 1)   n = 1;
	if(n > 600) n = 600;
	console_print("gnss_jmon: %d readings, 1 s apart (console busy until done)\n", n);
	{
		int32_t g1, g2;
		int ok = gnss_jam_read_gain(&g1, &g2);
		gnss_pt_jam_monitor((uint32_t)n, 1000U, g1, g2, ok);
	}
}

void set_gnss_jcal(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_jcal= needs 1 (capture at broadside) or 0 (clear)\n");
		return;
	}
	if(gnss_pt_jam_calibrate(((int)param[0] != 0) ? 1 : 0) == GNSS_PT_ERR_JAM_MISSING)
		console_print("gnss_jcal: jammer estimator not in this bitstream\n");
}

void set_gnss_jthr(double* param, char param_no)
{
	long db;

	if(param_no < 1) {
		console_print("gnss_jthr= needs a JNR in dB, e.g. gnss_jthr=6\n");
		return;
	}
	db = (long)param[0];
	if(db < 1 || db > 40) {
		console_print("gnss_jthr: %d dB out of range (1..40)\n", db);
		return;
	}
	if(gnss_pt_jam_set_threshold((int)db) == GNSS_PT_ERR_JAM_MISSING)
		console_print("gnss_jthr: jammer estimator not in this bitstream\n");
}

void set_gnss_jpcal(double* param, char param_no)
{
	int32_t g1, g2;
	double p;
	long p_c2;
	int32_t r;

	if(param_no < 1) {
		console_print("gnss_jpcal= needs the generator level in dBm, e.g. gnss_jpcal=-60\n");
		return;
	}
	if(!gnss_jam_read_gain(&g1, &g2)) {
		console_print("gnss_jpcal: cannot read the RX gain; radio not initialised?\n");
		return;
	}
	p = param[0];
	if(p > 0.0)
		p = -p;            /* RX inputs never see positive dBm; 60 means -60 */
	if(p < -130.0 || p > -10.0) {
		console_print("gnss_jpcal: %d dBm is outside -130..-10 dBm\n", (long)p);
		return;
	}
	p_c2 = (long)(p * 100.0 + ((p >= 0.0) ? 0.5 : -0.5));
	r = gnss_pt_jam_power_calibrate(p_c2, g1, g2);
	if(r == GNSS_PT_ERR_JAM_MISSING)
		console_print("gnss_jpcal: jammer estimator not in this bitstream\n");
}

void set_gnss_jinv(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_jinv= needs 0 or 1\n");
		return;
	}
	if(gnss_pt_jam_set_invert(((int)param[0] != 0) ? 1 : 0) == GNSS_PT_ERR_JAM_MISSING)
		console_print("gnss_jinv: jammer estimator not in this bitstream\n");
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-13/14. Input level (RX gain) and MVDR output gain.
 *
 * The order that gives the best C/N0 with and without a jammer:
 *   1. jammer OFF: gnss_rxgain=N until gnss_rxgain? shows a noise floor of
 *      about -60 dBFS (gnss_rxauto=1 does it). That keeps ~57 dB between the noise floor and ADC
 *      clipping, so a jammer up to ~45 dB JNR is still linear and can be
 *      nulled. A clipped jammer cannot be nulled by any weight.
 *   2. gnss_aolc=24 : the MVDR output level is held at -24 dBFS
 *      automatically (MVDR >= 1.10). gnss_aolc=0 / gnss_ogain? for the
 *      manual 2^n gain. Retune tx1_attenuation= if the receiver needs it.
 *   3. jammer ON: gnss_mvdr? -- ADC clipping must stay 0.
*******************************************************************************/
void get_gnss_rxgain(double* param, char param_no)
{
	uint8_t m1 = 0xFF, m2 = 0xFF;
	int32_t g1 = 0, g2 = 0;

	if(!ad9361_phy) {
		console_print("gnss_rxgain: ad9361_phy is NULL\n");
		return;
	}
	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &m1);
	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &m2);
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	console_print("GNSS_RXGAIN: RX1 %d dB (%s), RX2 %d dB (%s)\n",
		      (long)g1, (char *)gc_mode_name(m1),
		      (long)g2, (char *)gc_mode_name(m2));
	(void)gnss_check_matched_gain();
	gnss_pt_mvdr_print_levels();
}

void set_gnss_rxgain(double* param, char param_no)
{
	double p[1];
	long g;

	if(param_no < 1) {
		console_print("gnss_rxgain= needs a gain in dB, e.g. gnss_rxgain=30\n");
		return;
	}
	if(!ad9361_phy) {
		console_print("gnss_rxgain: ad9361_phy is NULL\n");
		return;
	}
	g = (long)param[0];
	if(g < 0 || g > GNSS_RX_GAIN_MAX_DB) {
		console_print("gnss_rxgain: %d dB out of range (0..%d)\n",
			      g, (long)GNSS_RX_GAIN_MAX_DB);
		return;
	}
	/* Make sure the plan is in FIRST, so a later gnss_mvdr=1 does not
	 * overwrite the gain chosen here. */
	gnss_radio_ensure_defaults();

	p[0] = 0;          set_rx1_gc_mode(p, 1);
	p[0] = 0;          set_rx2_gc_mode(p, 1);
	p[0] = (double)g;  set_rx1_rf_gain(p, 1);
	p[0] = (double)g;  set_rx2_rf_gain(p, 1);

	/* Let the 8192-sample window (182 us at 45 MSPS) refill at the new gain,
	 * and the jammer estimator average (~1.5 ms) settle. */
	no_os_mdelay(5);
	gnss_guard_set_op_gain((int32_t)g);   /* MOD-15c: the clip guard returns here */
	get_gnss_rxgain(param, param_no);
	console_print("  With AOLC on (gnss_aolc?) the TX1 level does not change with the\n"
		      "  RX gain. AOLC off: the MVDR output moves by the same dB.\n");
}

void get_gnss_ogain(double* param, char param_no)
{
	gnss_pt_mvdr_print_out_gain();
}

void set_gnss_ogain(double* param, char param_no)
{
	long n;
	int32_t r;

	if(param_no < 1) {
		console_print("gnss_ogain= needs 0..7 (gain 2^n, +6 dB per step)\n");
		return;
	}
	n = (long)param[0];
	if(n < 0 || n > (long)GNSS_PT_MVDR_OUT_GAIN_MAX) {
		console_print("gnss_ogain: %d out of range (0..%d)\n",
			      n, (long)GNSS_PT_MVDR_OUT_GAIN_MAX);
		return;
	}
	r = gnss_pt_mvdr_set_out_gain((uint32_t)n);
	if(r == GNSS_PT_ERR_MVDR_ID) {
		console_print("gnss_ogain: MVDR core not in this bitstream\n");
		return;
	}
	if(r == GNSS_PT_ERR_MVDR_OLD) {
		console_print("gnss_ogain: needs MVDR window 1.4 (OUT_GAIN); reprogram the FPGA\n");
		return;
	}
	if(r != GNSS_PT_OK) {
		console_print("gnss_ogain: commit failed; OUT_GAIN not applied\n");
		return;
	}
	console_print("GNSS_OGAIN: OUT_GAIN = %d (x%d, +%d dB on the MVDR path)\n",
		      n, (long)(1L << n), 6L * n);
	no_os_mdelay(2);
	gnss_pt_mvdr_print_out_gain();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-14. MVDR block v1.12 features.
 *
 *   gnss_aolc=T : automatic output level control. Holds the MVDR output noise
 *                 floor at -T dBFS (T = 6..40, 18 recommended) with a smooth
 *                 0.02 dB-step gain, so the 12-bit output and the DAC/TX noise
 *                 cost no C/N0 whatever the RX gain. gnss_aolc=0 returns to
 *                 the manual 2^n gain (gnss_ogain=). TX1 gets LOUDER when it
 *                 is enabled (up to ~45 dB), so the TX attenuation is driven
 *                 to the maximum first and the operating value restored after.
 *   gnss_iq=1/0/2 : receiver IQ-image correction on / off / clear and restart.
 *   gnss_coh=N  : MVDR jammer detection threshold, coherence > 2^-N. Lower it
 *                 below the jammer-OFF COH_L2 reading (gnss_mvdr?) on a bench
 *                 where RX1 and RX2 share one signal through a splitter.
*******************************************************************************/
void get_gnss_aolc(double* param, char param_no)
{
	gnss_pt_mvdr_print_out_gain();
}

void set_gnss_aolc(double* param, char param_no)
{
	long t;
	int32_t r;
	uint32_t a = 0;
	int restore = 0;

	if(param_no < 1) {
		console_print("gnss_aolc= needs a target in dB below full scale (6..40), or 0 = off\n");
		return;
	}
	t = (long)param[0];
	if(t < 0) t = -t;                 /* gnss_aolc=-18 means the same as 18 */
	if(t != 0 && (t < 6 || t > 40)) {
		console_print("gnss_aolc: %d dB out of range (6..40, or 0 = off)\n", t);
		return;
	}
	/* Quietest first: enabling AOLC raises the MVDR output level. */
	if(t != 0 && ad9361_phy &&
	   ad9361_get_tx_attenuation(ad9361_phy, 0, &a) == 0 &&
	   a < (uint32_t)GNSS_L1_TX_MAX_ATTEN_MDB) {
		if(gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB) == GNSS_L1_OK)
			restore = 1;
	}
	r = gnss_pt_mvdr_set_aolc((uint32_t)t);
	if(r == GNSS_PT_ERR_MVDR_ID) {
		console_print("gnss_aolc: MVDR core not in this bitstream\n");
	} else if(r == GNSS_PT_ERR_MVDR_OLD) {
		console_print("gnss_aolc: needs MVDR window 1.10; reprogram the FPGA\n");
	} else if(r != GNSS_PT_OK) {
		console_print("gnss_aolc: commit failed; not applied\n");
	} else if(t == 0) {
		console_print("GNSS_AOLC: OFF - manual output gain (gnss_ogain=) is back in use\n");
	} else {
		console_print("GNSS_AOLC: ON  - MVDR output noise floor held at -%d dBFS\n", t);
		console_print("           (settles in ~0.1 s; the gain moves in 0.02 dB steps)\n");
	}
	if(restore) {
		no_os_mdelay(200);        /* let the loop converge before opening TX */
		(void)gnss_l1_set_tx_attenuation(ad9361_phy, a);
		console_print("           TX1 attenuation back to %d mdB. If the receiver is\n"
			      "           now too hot, raise it (bigger = quieter).\n", (long)a);
	}
	gnss_pt_mvdr_print_out_gain();
}

void get_gnss_iq(double* param, char param_no)
{
	gnss_pt_mvdr_print_iq();
}

void set_gnss_iq(double* param, char param_no)
{
	long m;
	int32_t r;

	if(param_no < 1) {
		console_print("gnss_iq= needs 1 (on), 0 (off) or 2 (clear and restart)\n");
		return;
	}
	m = (long)param[0];
	if(m < 0 || m > 2) {
		console_print("gnss_iq: %d not valid (0, 1 or 2)\n", m);
		return;
	}
	r = gnss_pt_mvdr_set_iq((int)m);
	if(r == GNSS_PT_ERR_MVDR_ID) {
		console_print("gnss_iq: MVDR core not in this bitstream\n");
		return;
	}
	if(r == GNSS_PT_ERR_MVDR_OLD) {
		console_print("gnss_iq: needs MVDR window 1.9; reprogram the FPGA\n");
		return;
	}
	if(r != GNSS_PT_OK) {
		console_print("gnss_iq: commit failed; not applied\n");
		return;
	}
	gnss_pt_mvdr_print_iq();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-21 / MOD-24. Null core / profile (same bitstream).
 *   0 = MVDR: one complex weight per antenna, loading x1 noise, null hold
 *       0.3 s + glide 0.3 s
 *   1 = plain power inversion: one weight, loading x1/8, no hold, no glide
 *   2 = STAP (MOD-24, MVDR window >= 1.13, DEFAULT): space-time power
 *       inversion, 2 antennas x 5 taps, adapted every sample. Nulls CW AND
 *       wideband / barrage jammers: the 5-tap FIRs equalise the RX1/RX2
 *       difference across the band that limits one weight (MOD-23: 2.8 dB
 *       of null on a 56 MHz barrage). The single-weight core keeps running
 *       for the reports (coherence, JNR, NF_EST); only the output changes.
 * 0/1 use w = R^-1 [1,0]^T; IQ correction follows gnss_iq= (default OFF).
 * Written and committed at once.
*******************************************************************************/
static void gnss_null_print(void)
{
	uint32_t sm = gnss_pt_read(GNSS_PT_REG_MVDR_SMOOTH);
	int p = gnss_pt_get_profile();
	console_print("GNSS_NULL: %d = %s\n", (long)p,
		      (p == 3) ? "SMI-MVDR STAP, sample-matrix inversion, 2 antennas x L taps + 2 image" :
		      (p == 2) ? "STAP, space-time power inversion (CW + wideband/barrage)" :
		      (p == 1) ? "PLAIN POWER INVERSION (one weight, light loading, no hold/glide)"
			       : "MVDR (one weight, noise loading x1, null hold + glide)");
	if (p == 3) {
		if (gnss_smi_available()) {
			gnss_smi_state_t ss;
			gnss_smi_get_state(&ss);
			console_print("  SMI %s, 2 x %d taps + 2 image (N %d), window 2^%d, %d weight sets solved. gnss_smi? for its state.\n",
				      ss.hw_active ? "ACTIVE (drives the output)" : "NOT active yet",
				      (long)ss.La, (long)ss.Na, (long)ss.kexp, (long)ss.updates);
		} else {
			console_print("  This bitstream has no SMI-STAP (MVDR window < 1.14): running as %d.\n"
				      "  Program the 1.14 bitstream for SMI.\n", (long)gnss_pt_profile_effective());
		}
	}
	if (p == 2) {
		if (gnss_pt_stap_available()) {
			uint32_t st = gnss_pt_read(GNSS_PT_REG_STAP_ST);
			console_print("  STAP %s, mu 2^-%d. gnss_stap? for its state, gnss_supp? for the null.\n",
				      (st & GNSS_PT_STAP_ST_ACTIVE) ? "ACTIVE (drives the output)" : "NOT active yet",
				      (long)gnss_pt_get_stap_mu());
		} else {
			console_print("  This bitstream has no NLMS STAP: running as 0 (MVDR).\n"
				      "  gnss_null=3 (SMI-STAP) is the better core on 1.14.\n");
		}
	}
	console_print("  single-weight core: EPS_NF k=%d (x%d/8 noise)  HOLD %d  REL_SH %d  COH_SH %d  IQ %s\n",
		      (long)(gnss_pt_read(GNSS_PT_REG_MVDR_EPS_NF) & 0xFFU),
		      (long)(gnss_pt_read(GNSS_PT_REG_MVDR_EPS_NF) & 0xFFU),
		      (long)(sm & 0xFFFFU), (long)((sm >> 16) & 0xFU), (long)((sm >> 24) & 0xFU),
		      (gnss_pt_read(GNSS_PT_REG_MVDR_IQ_CTRL) & GNSS_PT_IQ_OFF) ? "OFF" : "ON");
}

void get_gnss_null(double* param, char param_no)
{
	gnss_null_print();
}

void set_gnss_null(double* param, char param_no)
{
	int p;
	if(param_no < 1 || param[0] < 0 || param[0] > 3) {
		console_print("gnss_null= needs 0 (MVDR), 1 (plain power inversion), 2 (STAP NLMS) or 3 (SMI-MVDR STAP)\n");
		return;
	}
	if(gnss_pt_read(GNSS_PT_REG_MVDR_ID) != GNSS_PT_MVDR_EXPECTED_ID) {
		console_print("gnss_null: MVDR core not in this bitstream\n");
		return;
	}
	p = (int)param[0];
	if (p == 2 && !gnss_pt_stap_available()) {
		console_print("gnss_null=2: this bitstream has no NLMS STAP (older than 1.13, or a 1.14\n"
			      "  SMI-only build); kept, but it runs as 0 (MVDR). Use gnss_null=3 (SMI).\n");
	}
	if (p == 3 && !gnss_smi_available()) {
		console_print("gnss_null=3: this bitstream has no SMI-STAP (MVDR window < 1.14);\n"
			      "  kept, but it runs as %d until the 1.14 bitstream is loaded.\n",
			      (long)(gnss_pt_stap_available() ? 2 : 0));
	}
	gnss_pt_set_profile(p);
	gnss_pt_mvdr_write_defaults();
	if(gnss_pt_mvdr_commit() != GNSS_PT_OK) {
		console_print("gnss_null: commit failed; not applied\n");
		return;
	}
	gnss_null_print();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-25. SMI-MVDR STAP (gnss_null=3).
 *   gnss_smi?            state (gnss_smi.c, gnss_smi_print)
 *   gnss_smi=k[,alpha[,dc]] window 2^k snapshots, and
 *                        diagonal loading alpha x lambda_min (noise per tap).
 *                        Larger k: deeper, steadier null, slower reaction
 *                        (8 taps: one every ~24 samples, k 9 = 0.23 ms; 16 taps:
 *                        one every ~75 samples, k 8 = 0.43 ms at 45 MSPS). Larger
 *                        alpha: less noise cost, shallower null.
 *   gnss_steer=theta     unity gain + array gain toward theta (deg from
 *                        broadside, same sign/calibration as gnss_jam?)
 *   gnss_steer=az,el,b   satellite azimuth/elevation, b = compass bearing of
 *                        the array's +90 deg (end-fire) direction
 *   gnss_steer=999       back to power inversion (default; all satellites)
 *   gnss_lookph=phi      steer to a raw RX2-RX1 phase (degrees)
 * Steering protects ONE direction better (+~2..3 dB) but a 2-element null
 * then also has to keep that direction: jammers near it are nulled less.
 * Power inversion (default) is the right choice for a GNSS receiver that
 * tracks many satellites.
*******************************************************************************/
void get_gnss_smi(double* param, char param_no)
{
	gnss_smi_print();
}

void set_gnss_smi(double* param, char param_no)
{
	if (param_no < 1 || gnss_smi_set_kexp((uint32_t)param[0]) != 0) {
		console_print("gnss_smi= needs the window exponent k 6..12 [, loading alpha 0.5..1000]\n");
		return;
	}
	if (param_no >= 2 && gnss_smi_set_alpha(param[1]) != 0) {
		console_print("gnss_smi: loading alpha must be 0.5..1000; kept\n");
	}
	if (param_no >= 3) {
		gnss_smi_set_dc(param[2] != 0);      /* FPGA DC removal at the SMI input */
	}
	gnss_smi_print();
}

void get_gnss_steer(double* param, char param_no)
{
	gnss_smi_state_t s;
	gnss_smi_get_state(&s);
	if (!s.steer) {
		console_print("GNSS_STEER: off -- SMI power inversion (unity gain on RX1, all directions).\n");
		return;
	}
	{
		long t = (long)(10.0 * ((s.steer_phi_deg < 0) ? -s.steer_phi_deg : s.steer_phi_deg) + 0.5);
		console_print("GNSS_STEER: RX2-RX1 phase %s%d.%d deg", (s.steer_phi_deg < 0) ? "-" : "",
			      t / 10, t % 10);
	}
	if (s.steer_theta < 900.0) {
		console_print(" (theta %d deg from broadside)", (long)s.steer_theta);
	}
	console_print("\n");
}

void set_gnss_steer(double* param, char param_no)
{
	int r;
	gnss_steer_user_changed();       /* MOD-26b: ends a jammer suspension */
	if (param_no < 1 || param[0] >= 900.0 || param[0] <= -900.0) {
		gnss_smi_steer_off();
		get_gnss_steer(param, 0);
		return;
	}
	if (param_no >= 3) {
		r = gnss_smi_steer_azel(param[0], param[1], param[2]);
	} else {
		r = gnss_smi_steer_theta(param[0]);
	}
	if (r != 0) {
		console_print("gnss_steer= needs theta -90..90, or az,el(0..90),bearing; 999 = off\n");
		return;
	}
	if (gnss_pt_get_profile() != 3) {
		console_print("  (applies to SMI only: gnss_null=3)\n");
	}
	get_gnss_steer(param, 0);
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-25. gnss_check? / gnss_cmds? / gnss_color= (gnss_dash.c)
*******************************************************************************/
void get_gnss_check(double* param, char param_no) { gnss_dash_check(); }
void get_gnss_report(double* param, char param_no) { gnss_rep_print(); }   /* MOD-26 */

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-26: the operator's steering modules (gnss_mvdr_steer.c,
 * gnss_mvdr_auto.c). They steer the single-weight MVDR (A1/A2); gnss_beam=2
 * also steers SMI to broadside while no jammer is present (gnss_autostart.c).
*******************************************************************************/
void get_gnss_floor(double* param, char param_no) { gnss_floor_scan(); }   /* MOD-27 */
void get_gnss_kpi(double* param, char param_no) { gnss_rep_print_kpi(); }    /* MOD-28 */
void get_gnss_spec(double* param, char param_no) { gnss_spec_scan(-1); }    /* MOD-29 */
void set_gnss_spec(double* param, char param_no)                          /* MOD-29 */
{
	if (param_no < 1 || param[0] < 0.0) {
		console_print("gnss_spec= needs a gain in dB, e.g. gnss_spec=0 (board floor)\n");
		return;
	}
	gnss_spec_scan((int)(param[0] + 0.5));
}
void get_gnss_bist(double* param, char param_no) { gnss_bist_test(); }      /* MOD-29 */
void get_gnss_beam(double* param, char param_no) { gnss_mvdr_auto_print(); }
void set_gnss_beam(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_beam= needs 0 (off), 1 (auto, omni without jammer) or 2 (auto, broadside without jammer)\n");
		return;
	}
	gnss_mvdr_auto_enable((int)param[0], 1);
	if(gnss_pt_get_profile() == 3)
		console_print("  SMI: %s\n", ((int)param[0] == 2)
			      ? "steered to broadside while no jammer is present, power inversion while jammed"
			      : "power inversion (with 2 antennas the noise-subspace beam gives the same satellite SNR)");
}
void get_gnss_mvsteer(double* param, char param_no)
{
	uint32_t a1 = 0, a2 = 0;
	int src = 0, have = 0;
	long cdeg = 0;
	gnss_pt_steer_get(&a1, &a2, &src, &cdeg, &have);
	console_print("GNSS_MVSTEER: %s, A1 0x%08x A2 0x%08x", (char *)gnss_pt_steer_src_name(src),
		      (long)a1, (long)a2);
	if(have)
		console_print(", main lobe %s%d.%02d deg", (cdeg < 0) ? "-" : "",
			      (long)((cdeg < 0 ? -cdeg : cdeg) / 100), (long)((cdeg < 0 ? -cdeg : cdeg) % 100));
	console_print("\n");
}
void set_gnss_mvsteer(double* param, char param_no)
{
	if(param_no < 1 || param[0] >= 900.0 || param[0] <= -900.0) {
		(void)gnss_pt_steer_reset();
		get_gnss_mvsteer(param, 0);
		return;
	}
	if(param[0] < -90.0 || param[0] > 90.0) {
		console_print("gnss_mvsteer= needs theta -90..90 deg (999 = power inversion)\n");
		return;
	}
	(void)gnss_mvdr_set_steering(param[0], NAN, 0, 1);
	if(gnss_pt_get_profile() == 3)
		console_print("  (single-weight MVDR only: SMI is running, use gnss_steer= for SMI)\n");
}
void get_gnss_cmds(double* param, char param_no)  { gnss_dash_cmds(); }
void set_gnss_color(double* param, char param_no)
{
	if (param_no < 1) {
		console_print("gnss_color= needs 1 (colours) or 0 (plain text)\n");
		return;
	}
	gnss_color_set(param[0] != 0);
	console_print("GNSS_COLOR: %s%s%s\n", gnss_c(GC_GRN),
		      gnss_color_get() ? "ON (ANSI colours)" : "OFF (plain text)", gnss_c(GC_RST));
}

void set_gnss_lookph(double* param, char param_no)
{
	if (param_no < 1 || param[0] < -360.0 || param[0] > 360.0) {
		console_print("gnss_lookph= needs a phase in degrees (-360..360)\n");
		return;
	}
	gnss_smi_steer_phase(param[0]);
	get_gnss_steer(param, 0);
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-24. gnss_supp? -- the jammer suppression report.
 * THE "print suppression" IS ONLY A PRINT (display) FIGURE:
 *     (jammer + noise in) / (jammer residual + noise out)
 * The noise is never cancelled, so it stops at the JNR at the ADC (printed
 * next to it). The ACTUAL cancellation (jammer in / jammer residual) is the
 * real null depth. All three are drawn on a 0..72 dB bar = the ideal 12-bit
 * range (6.02 dB x 12). Why the print is ~35..40 dB and not 72 on this board
 * (AD9361 noise floor ~-47 dBFS, jammer peaks kept under full scale): see the
 * long comment above gnss_pt_supp_compute() in gnss_passthrough.c and the
 * "why not 72 dB" line of the report itself.
*******************************************************************************/
void get_gnss_supp(double* param, char param_no)
{
	(void)gnss_pt_print_suppression();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-24. gnss_stap? / gnss_stap=mu
*******************************************************************************/
void get_gnss_stap(double* param, char param_no)
{
	uint32_t ctrl, st, wn, ns;
	if (!gnss_pt_stap_available()) {
		console_print("NLMS STAP: not in this bitstream (older than 1.13, or 1.14 SMI-only build: use gnss_null=3).\n");
		return;
	}
	ctrl = gnss_pt_read(GNSS_PT_REG_STAP_CTRL);
	st   = gnss_pt_read(GNSS_PT_REG_STAP_ST);
	wn   = gnss_pt_read(GNSS_PT_REG_STAP_WNORM);
	ns   = gnss_pt_read(GNSS_PT_REG_STAP_NWSAT);
	console_print("STAP: %s (gnss_null=%d), enable bit %d\n",
		      (st & GNSS_PT_STAP_ST_ACTIVE) ? "ACTIVE, drives the output" : "off (single-weight output)",
		      (long)gnss_pt_get_profile(), (long)(ctrl & 1U));
	console_print("  step mu 2^-%d, leak 2^-%d, PL %d (input power ~2^%d in x4 units)\n",
		      (long)((ctrl >> 4) & 0xFU), (long)((ctrl >> 8) & 0x1FU),
		      (long)(st & 0x3FU), (long)((long)(st & 0x3FU) - 2));
	console_print("  sum |w|^2 = %d/1000 -> output noise x(1 + that): the noise cost of the null\n",
		      (long)(((uint64_t)wn * 1000ULL) >> 24));
	console_print("  anti-windup events %d %s\n", (long)ns,
		      ns ? "(weights hit +/-16: extreme jammer or clipping; check gnss_agc?)" : "(none: good)");
}

void set_gnss_stap(double* param, char param_no)
{
	if (param_no < 1 || param[0] < 5 || param[0] > 10) {
		console_print("gnss_stap= needs the step mu, 5..10 (default 5; 4 and less diverge)\n");
		return;
	}
	gnss_pt_set_stap_mu((uint32_t)param[0]);
	if (!gnss_pt_stap_available()) {
		console_print("gnss_stap: kept (mu %d), but this bitstream has no STAP\n", (long)gnss_pt_get_stap_mu());
		return;
	}
	gnss_pt_mvdr_write_defaults();
	if (gnss_pt_mvdr_commit() != GNSS_PT_OK) {
		console_print("gnss_stap: commit failed; not applied\n");
		return;
	}
	get_gnss_stap(param, 0);
}

void set_gnss_coh(double* param, char param_no)
{
	long n;
	int32_t r;

	if(param_no < 1) {
		console_print("gnss_coh= needs 1..15 (default 4)\n");
		return;
	}
	n = (long)param[0];
	if(n < 1 || n > 15) {
		console_print("gnss_coh: %d out of range (1..15)\n", n);
		return;
	}
	r = gnss_pt_mvdr_set_coh((uint32_t)n);
	if(r == GNSS_PT_ERR_MVDR_ID) {
		console_print("gnss_coh: MVDR core not in this bitstream\n");
		return;
	}
	if(r == GNSS_PT_ERR_MVDR_OLD) {
		console_print("gnss_coh: needs MVDR window 1.8; reprogram the FPGA\n");
		return;
	}
	if(r != GNSS_PT_OK) {
		console_print("gnss_coh: commit failed; not applied\n");
		return;
	}
	console_print("GNSS_COH: jammer detected when RX1/RX2 coherence > 2^-%d.\n"
		      "          With the jammer OFF, gnss_mvdr? COH_L2 must read ABOVE %d.\n",
		      n, n);
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-15. Automatic start / health / RX auto-level.
 *   gnss_auto=1   : the same sequence as at boot (radio plan, MVDR, RX gain
 *                   auto-level, AOLC, TX1 opened last), with a PASS / WARN /
 *                   FAIL line per step and a summary.
 *   gnss_auto=0   : TX1 silent (same as gnss_tx=0).
 *   gnss_health?  : every check of the boot report, read-only.
 *   gnss_rxauto=1 : only the RX gain auto-level (TX1 kept where it is: the
 *                   output level is held by AOLC while the gain moves).
*******************************************************************************/
void set_gnss_auto(double* param, char param_no)
{
	double p[1];

	if(param_no < 1) {
		console_print("gnss_auto= needs 1 (start) or 0 (TX1 silent)\n");
		return;
	}
	if((int)param[0] == 0) {
		p[0] = 0;
		set_gnss_tx(p, 1);
		return;
	}
	(void)gnss_autostart_run();
}

void get_gnss_health(double* param, char param_no)
{
	(void)gnss_health_check(1);
}

void get_gnss_guard(double* param, char param_no)
{
	gnss_guard_print();
}

void set_gnss_guard(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_guard= needs 1 (on) or 0 (off)\n");
		return;
	}
	gnss_guard_enable((int)param[0] != 0);
	gnss_guard_print();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-16. MVDR-aware versions of the vendor rx1/rx2 gain and
 *        gain-mode commands (a PC tool sends them in RX1/RX2 pairs).
 *
 * While MVDR is running on TX1 the two channels MUST stay manual and equal:
 *   rxN_rf_gain=G  -> sets RX1 AND RX2 to G (and makes G the operating gain
 *                     the clip guard returns to)
 *   rxN_gc_mode=1/2/3 (an AGC) -> refused: two AGCs move RX1 and RX2
 *                     independently and break the null. Instead the
 *                     MVDR-safe automatic gain is used: the auto-level gain
 *                     plus the clip guard (it lowers the gain when a jammer
 *                     clips the ADC and restores it afterwards).
 *   rxN_gc_mode=0  -> manual (both channels); the gain stays where it is.
 * With MVDR off they behave exactly like the vendor commands.
*******************************************************************************/
static int gnss_mvdr_running(void)
{
	uint32_t st = gnss_pt_read(GNSS_PT_REG_STATUS);
	return (gnss_pt_read(GNSS_PT_REG_MVDR_ID) == GNSS_PT_MVDR_EXPECTED_ID) &&
	       (st & GNSS_PT_ST_MVDR_SEL_SYNCED) ? 1 : 0;
}

static void gnss_set_rx_gc_common(double* param, char param_no, int ch)
{
	double p[1];
	long m;

	if(param_no < 1 || !gnss_mvdr_running()) {
		if(ch == 1) set_rx1_gc_mode(param, param_no);
		else        set_rx2_gc_mode(param, param_no);
		return;
	}
	m = (long)param[0];
	p[0] = 0;
	set_rx1_gc_mode(p, 1);
	set_rx2_gc_mode(p, 1);
	if(m == 0) {
		if(ch == 1) {
			gnss_agc_set_mode(0);
			console_print("  (MVDR: RX gain MANUAL at the current gain, RX1 = RX2; clipping\n"
				      "   still protected. rx1_gc_mode=2 or gnss_agc=1 = AUTO.)\n");
		}
		return;
	}
	if(ch == 2)
		return;                 /* the RX1 command already did it all */
	console_print("  MVDR is running: an AD9361 AGC (mode %d) would move RX1 and RX2\n"
		      "  independently and break the null. Using the MVDR-safe AUTO gain.\n", m);
	gnss_guard_auto();
}

void gnss_set_rx1_gc_mode(double* param, char param_no) { gnss_set_rx_gc_common(param, param_no, 1); }
void gnss_set_rx2_gc_mode(double* param, char param_no) { gnss_set_rx_gc_common(param, param_no, 2); }

static void gnss_set_rx_gain_common(double* param, char param_no, int ch)
{
	double p[1];
	int32_t g1 = -999, g2 = -999;
	long g;

	if(param_no < 1 || !gnss_mvdr_running()) {
		if(ch == 1) set_rx1_rf_gain(param, param_no);
		else        set_rx2_rf_gain(param, param_no);
		return;
	}
	g = (long)param[0];
	if(g < GNSS_CFG_RX_GAIN_MIN_DB) g = GNSS_CFG_RX_GAIN_MIN_DB;
	if(g > GNSS_CFG_RX_GAIN_MAX_DB) g = GNSS_CFG_RX_GAIN_MAX_DB;
	ad9361_get_rx_rf_gain(ad9361_phy, 0, &g1);
	ad9361_get_rx_rf_gain(ad9361_phy, 1, &g2);
	if(g1 == g && g2 == g) {          /* the pair's second half: already set */
		console_print("rx%d_rf_gain=%d\n", (long)ch, g);
		return;
	}
	p[0] = 0;          set_rx1_gc_mode(p, 1);  set_rx2_gc_mode(p, 1);
	p[0] = (double)g;  set_rx1_rf_gain(p, 1);  set_rx2_rf_gain(p, 1);
	gnss_guard_set_op_gain((int32_t)g);
	console_print("  (MVDR: RX1 and RX2 both %d dB; operating gain for the clip guard)\n", g);
}

void gnss_set_rx1_rf_gain(double* param, char param_no) { gnss_set_rx_gain_common(param, param_no, 1); }
void gnss_set_rx2_rf_gain(double* param, char param_no) { gnss_set_rx_gain_common(param, param_no, 2); }

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-17. Radio presets and custom plans.
 *   gnss_band?        lists the presets
 *   gnss_band=N       selects preset N and re-runs the automatic start
 *   gnss_plan=LO,BW,FS[,G[,A]]  any plan: LO MHz, RF bandwidth MHz, sample
 *                     rate MSPS, starting RX gain dB, TX1 attenuation mdB
 *   tx1_attenuation=  also becomes the operating value kept by gnss_auto=1
*******************************************************************************/
static void gnss_print_plan_line(int n, uint64_t lo, uint32_t bw, uint32_t fs,
				 int32_t g, uint32_t a, const char *name, int now)
{
	console_print("  %c %d  LO %d.%d MHz  BW %d MHz  %d.%d MSPS  start gain %d dB  TX1 %d mdB  %s\n",
		      now ? '*' : ' ', (long)n,
		      (long)(lo / 1000000ULL), (long)((lo % 1000000ULL) / 10000ULL),
		      (long)(bw / 1000000U),
		      (long)(fs / 1000000U), (long)((fs % 1000000U) / 100000U),
		      (long)g, (long)a, (char *)name);
}

void get_gnss_band(double* param, char param_no)
{
	int n;
	console_print("GNSS_BAND: presets (gnss_band=N), * = in use\n");
	for(n = 0; n < GNSS_NUM_BANDS; n++)
		gnss_print_plan_line(n, gnss_bands[n].lo_hz, gnss_bands[n].bw_hz,
				     gnss_bands[n].fs_hz, gnss_bands[n].gain_db,
				     gnss_bands[n].atten_mdb, gnss_bands[n].name,
				     n == gnss_band_now);
	if(gnss_band_now < 0)
		gnss_print_plan_line(-1, gnss_lo_hz, gnss_bw_hz, gnss_fs_hz, gnss_gain0_db,
				     gnss_plan_atten_mdb, "custom (gnss_plan=)", 1);
	console_print("  l_clk = 4 x rate must stay <= 250 MHz (62.5 MSPS max).\n");
	console_print("  signals with the plan in use:\n");
	gnss_rep_print_signals(1);                      /* MOD-26 */
}

void set_gnss_band(double* param, char param_no)
{
	long n;
	if(param_no < 1) {
		get_gnss_band(param, param_no);
		return;
	}
	n = (long)param[0];
	if(gnss_select_band((int)n) != 0) {
		console_print("gnss_band: %d is not a preset (0..%d). gnss_band? lists them.\n",
			      n, (long)(GNSS_NUM_BANDS - 1));
		return;
	}
	console_print("GNSS_BAND: preset %d, %s -- restarting MVDR with it\n",
		      n, (char *)gnss_bands[n].name);
	gnss_rep_event("BAND        gnss_band=%ld, %s", n, gnss_bands[n].name);
	gnss_rep_print_signals(0);
	(void)gnss_autostart_run();
}

void set_gnss_plan(double* param, char param_no)
{
	double lo, bw, fs;

	if(param_no < 3) {
		console_print("gnss_plan= needs LO MHz, BW MHz, rate MSPS[, gain dB[, atten mdB]]\n"
			      "  e.g. gnss_plan=1575.42,40,45   or   gnss_plan=1584.5,51,51,50,40000\n");
		return;
	}
	lo = param[0] * 1e6;  bw = param[1] * 1e6;  fs = param[2] * 1e6;
	if(lo < 1.2e9 || lo > 1.65e9) {
		console_print("gnss_plan: LO must be 1200..1650 MHz (the GNSS bands)\n");
		return;
	}
	if(fs < 2.1e6 || fs > 61.44e6) {
		console_print("gnss_plan: rate must be 2.1..61.44 MSPS (AD9361, 2R2T LVDS)\n");
		return;
	}
	if(fs * 4.0 > (double)GNSS_PT_LCLK_CONSTRAINT_MHZ * 1e6) {
		console_print("gnss_plan: 4 x rate = l_clk would exceed the %d MHz the FPGA timing\n"
			      "  was closed at. Keep the rate <= %d.%d MSPS.\n",
			      (long)GNSS_PT_LCLK_CONSTRAINT_MHZ, (long)(GNSS_PT_LCLK_CONSTRAINT_MHZ / 4U),
			      (long)((GNSS_PT_LCLK_CONSTRAINT_MHZ % 4U) * 25U / 10U));
		return;
	}
	if(bw < 0.2e6 || bw > 56e6) {
		console_print("gnss_plan: RF bandwidth must be 0.2..56 MHz\n");
		return;
	}
	if(bw > fs * 1.0001)
		console_print("  NOTE: bandwidth > sample rate: the band edges alias. Usually BW <= rate.\n");
	gnss_lo_hz    = (uint64_t)(lo + 0.5);
	gnss_bw_hz    = (uint32_t)(bw + 0.5);
	gnss_fs_hz    = (uint32_t)(fs + 0.5);
	if(param_no >= 4) gnss_gain0_db = (int32_t)param[3];
	if(param_no >= 5) gnss_plan_atten_mdb = (uint32_t)param[4];
	gnss_band_now = -1;
	console_print("GNSS_PLAN: custom plan -- restarting MVDR with it\n");
	(void)gnss_autostart_run();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-20. MVDR-aware LO / sample rate / bandwidth commands.
 *
 * Seen in the field log: a PC tool set rx_samp_freq=61440000 while TX stayed
 * at 60 MSPS for a while -> RX 2.4 % faster than TX -> 3.2 million FIFO
 * overflows (2.4 % of the samples dropped) -> the GNSS receiver got a broken
 * stream. With MVDR running, these commands now change the PLAN (the same
 * value for RX AND TX), and the whole automatic start re-runs 1 s after the
 * last such command (so an rx/tx pair causes one restart, not two). With
 * MVDR off they are the vendor commands.
*******************************************************************************/
static int gnss_mvdr_running(void);

static void gnss_plan_edit(double lo_mhz, double bw_hz, double fs_hz, const char *name, double v)
{
	double lo = (lo_mhz > 0) ? lo_mhz * 1e6 : (double)gnss_lo_hz;
	double bw = (bw_hz  > 0) ? bw_hz       : (double)gnss_bw_hz;
	double fs = (fs_hz  > 0) ? fs_hz       : (double)gnss_fs_hz;

	if(lo < 1.2e9 || lo > 1.65e9 || fs < 2.1e6 || fs > 61.44e6 ||
	   fs * 4.0 > (double)GNSS_PT_LCLK_CONSTRAINT_MHZ * 1e6 || bw < 0.2e6 || bw > 56e6) {
		console_print("  %s: not applied -- outside the MVDR plan limits (LO 1200..1650 MHz,\n"
			      "  rate 2.1..62.5 MSPS, BW 0.2..56 MHz). gnss_plan= / gnss_band? \n", (char *)name);
		return;
	}
	gnss_lo_hz = (uint64_t)(lo + 0.5);
	gnss_bw_hz = (uint32_t)(bw + 0.5);
	gnss_fs_hz = (uint32_t)(fs + 0.5);
	gnss_band_now = -1;
	console_print("%s=%d\n", (char *)name, (long)v);
	console_print("  (MVDR: plan now LO %d Hz, BW %d Hz, %d SPS on RX AND TX;\n"
		      "   MVDR restarts with it in 1 s)\n",
		      (long)gnss_lo_hz, (long)gnss_bw_hz, (long)gnss_fs_hz);
	gnss_request_restart();
}

void gnss_set_rx_lo_freq(double* p, char n)      { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(p[0], 0, 0, "rx_lo_freq", p[0] * 1e6); else set_rx_lo_freq(p, n); }
void gnss_set_tx_lo_freq(double* p, char n)      { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(p[0], 0, 0, "tx_lo_freq", p[0] * 1e6); else set_tx_lo_freq(p, n); }
void gnss_set_rx_samp_freq(double* p, char n)    { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(0, 0, p[0], "rx_samp_freq", p[0]); else set_rx_samp_freq(p, n); }
void gnss_set_tx_samp_freq(double* p, char n)    { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(0, 0, p[0], "tx_samp_freq", p[0]); else set_tx_samp_freq(p, n); }
void gnss_set_rx_rf_bandwidth(double* p, char n) { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(0, p[0], 0, "rx_rf_bandwidth", p[0]); else set_rx_rf_bandwidth(p, n); }
void gnss_set_tx_rf_bandwidth(double* p, char n) { if(n >= 1 && gnss_mvdr_running()) gnss_plan_edit(0, p[0], 0, "tx_rf_bandwidth", p[0]); else set_tx_rf_bandwidth(p, n); }

void gnss_set_tx1_attenuation(double* param, char param_no)
{
	set_tx1_attenuation(param, param_no);
	if(param_no >= 1 && param[0] >= 0 && param[0] < (double)GNSS_L1_TX_MAX_ATTEN_MDB) {
		gnss_plan_atten_mdb = (uint32_t)param[0];
		gnss_op_atten_mdb   = (uint32_t)param[0];
	}
}

void get_gnss_agc(double* param, char param_no)
{
	gnss_agc_print();
}

void set_gnss_agc(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_agc= needs 1 (AUTO) or 0 (MANUAL)\n");
		return;
	}
	gnss_agc_set_mode((int)param[0] != 0);
	gnss_agc_print();
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-16. gnss_lo? / gnss_lo=MHz : LO at run time.
*******************************************************************************/
void get_gnss_lo(double* param, char param_no)
{
	uint64_t r = 0, t = 0;
	if(ad9361_phy) {
		ad9361_get_rx_lo_freq(ad9361_phy, &r);
		ad9361_get_tx_lo_freq(ad9361_phy, &t);
	}
	console_print("GNSS_LO: plan %d Hz, AD9361 RX %d Hz, TX %d Hz\n",
		      (long)gnss_lo_hz, (long)r, (long)t);
	console_print("         GPS L1 is at %d kHz from the LO\n",
		      (long)((1575420000LL - (long long)gnss_lo_hz) / 1000LL));
}

void set_gnss_lo(double* param, char param_no)
{
	uint64_t hz;

	if(param_no < 1) {
		console_print("gnss_lo= needs MHz, e.g. gnss_lo=1575.42 or gnss_lo=1584.5\n");
		return;
	}
	hz = (uint64_t)(param[0] * 1000000.0 + 0.5);
	if(hz < 1550000000ULL || hz > 1610000000ULL) {
		console_print("gnss_lo: %d Hz is outside 1550..1610 MHz\n", (long)hz);
		return;
	}
	gnss_lo_hz = hz;
	gnss_band_now = -1;
	console_print("GNSS_LO: %d Hz -- re-running the automatic start\n", (long)hz);
	(void)gnss_autostart_run();
}

void set_gnss_rxauto(double* param, char param_no)
{
	if(param_no < 1 || (int)param[0] != 1) {
		console_print("gnss_rxauto= needs 1\n");
		return;
	}
	(void)gnss_rx_autolevel(1);
}

/**************************************************************************//***
 * @brief Show the invalid parameter message.
 *
 * @return None.
*******************************************************************************/
void show_invalid_param_message(unsigned char cmd_no)
{
	console_print("Invalid parameter!\n");
	console_print("%s  - %s\n", (char*)cmd_list[cmd_no].name, (char*)cmd_list[cmd_no].description);
	console_print("Example: %s\n", (char*)cmd_list[cmd_no].example);
}

/**************************************************************************//***
 * @brief Displays all available commands.
 *
 * @return None.
*******************************************************************************/
void get_help(double* param, char param_no) // "help?" command
{
	unsigned char display_cmd;

	console_print("Available commands:\n");
	for(display_cmd = 0; display_cmd < cmd_no; display_cmd++)
	{
		console_print("%s  - %s\n", (char*)cmd_list[display_cmd].name,
								  (char*)cmd_list[display_cmd].description);
	}
}

/**************************************************************************//***
 * @brief Displays all available commands.
 *
 * @return None.
*******************************************************************************/
void get_register(double *param, char param_no)
{
    uint32_t reg_addr;
    int32_t reg_val;

    if (param_no >= 1)
    {
        reg_addr = (uint32_t)param[0];

        reg_val = ad9361_spi_read(ad9361_phy->spi, reg_addr);

        if (reg_val < 0)
        {
            console_print("SPI read failed: reg=0x%x error=%d\r\n",
                          (long)reg_addr,
                          (long)reg_val);
            return;
        }

        console_print("register[0x%x] = 0x%x\r\n",
                      (long)reg_addr,
                      (long)reg_val);

        console_print("Binary = %d%d%d%d%d%d%d%d\r\n",
                      (long)((reg_val >> 7) & 1),
                      (long)((reg_val >> 6) & 1),
                      (long)((reg_val >> 5) & 1),
                      (long)((reg_val >> 4) & 1),
                      (long)((reg_val >> 3) & 1),
                      (long)((reg_val >> 2) & 1),
                      (long)((reg_val >> 1) & 1),
                      (long)((reg_val >> 0) & 1));
    }
    else
    {
        show_invalid_param_message(1);
    }
}

/**************************************************************************//***
 * @brief Gets current TX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void get_tx_lo_freq(double* param, char param_no) // "tx_lo_freq?" command
{
	uint64_t lo_freq_hz;

	ad9361_get_tx_lo_freq(ad9361_phy, &lo_freq_hz);
#ifdef ANTSDR_E310
	/* set tx rf switch */
	if(lo_freq_hz <= 3000000000){
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_h   ,0);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_l   ,1);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_h   ,0);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_l   ,1);
		ad9361_set_tx_rf_port_output(ad9361_phy, TXB);
	}
	else {
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_h   ,1);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_l   ,0);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_h   ,1);
		no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_l   ,0);
		ad9361_set_tx_rf_port_output(ad9361_phy, TXA);
	}
#else
		ad9361_set_tx_rf_port_output(ad9361_phy, TXA);
#endif

	/* Print the exact Hz readback, not a truncated-to-whole-MHz value: this
	 * used to divide by 1e6 with INTEGER division before printing, so a real
	 * 1575419998 Hz LO always echoed back as "1575" regardless of precision
	 * elsewhere. The command still takes MHz as input (unchanged); only the
	 * confirmation is now exact Hz, matching gnss_status?'s "RX LO : ... Hz"
	 * and gnss_mvdr?'s "MVDR TXRX Parameters" block. */
	console_print("tx_lo_freq=%d\n", (long)lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the TX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void set_tx_lo_freq(double* param, char param_no) // "tx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no >= 1)
	{
		/* BUG FIX (found via gnss_mvdr=1 / gnss_tx=1 setting 1575.42 MHz for
		 * GPS L1): param[0] is a double MHz value. Assigning it straight into
		 * a uint64_t truncates any fractional MHz BEFORE the *1000000 scaling
		 * below, so 1575.42 silently became 1575 (a 420 kHz LO error) --
		 * confirmed on hardware: gnss_status? read back RX/TX LO = 1575000000
		 * Hz after gnss_tx=1, though rx_lo_freq=1575.42 / tx_lo_freq=1575.42
		 * were what was requested. Scale to Hz in floating point FIRST, then
		 * truncate -- 1575.42 * 1e6 = 1575420000.0 is exactly representable
		 * in a double (well under its 2^53 exact-integer range), so this
		 * loses nothing for any frequency this radio can tune to. */
		lo_freq_hz = (uint64_t)(param[0] * 1000000.0 + 0.5);   /* MOD-15: round, never truncate */
#ifdef ANTSDR_E310
		/* set tx rf switch */
		if(lo_freq_hz <= 3000000000){
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_h   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_l   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_h   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_l   ,1);
			ad9361_set_tx_rf_port_output(ad9361_phy, TXB);
		}
		else {
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_h   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_l   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_h   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_l   ,0);
			ad9361_set_tx_rf_port_output(ad9361_phy, TXA);
		}
#else
		ad9361_set_tx_rf_port_output(ad9361_phy, TXA);
#endif

		ad9361_set_tx_lo_freq(ad9361_phy, lo_freq_hz);
		/* Exact Hz, not truncated MHz -- see the comment in get_tx_lo_freq(). */
		console_print("tx_lo_freq=%d\n", (long)lo_freq_hz);
	}
}

/**************************************************************************//***
 * @brief Gets current sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_tx_samp_freq(double* param, char param_no) // "tx_samp_freq?" command
{
	uint32_t sampling_freq_hz;

	ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("tx_samp_freq=%d\n", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_tx_samp_freq(double* param, char param_no) // "tx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no >= 1)
	{
		sampling_freq_hz = (uint32_t)param[0];
		ad9361_set_tx_sampling_freq(ad9361_phy, sampling_freq_hz);
		ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);
//		dds_update(ad9361_phy);
		console_print("tx_samp_freq=%d\n", sampling_freq_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void get_tx_rf_bandwidth(double* param, char param_no) // "tx_rf_bandwidth?" command
{
	uint32_t bandwidth_hz;

	ad9361_get_tx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("tx_rf_bandwidth=%d\n", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void set_tx_rf_bandwidth(double* param, char param_no) // "tx_rf_bandwidth=" command
{
	uint32_t bandwidth_hz;

	if(param_no >= 1)
	{
		bandwidth_hz = param[0];
		ad9361_set_tx_rf_bandwidth(ad9361_phy, bandwidth_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX1 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void get_tx1_attenuation(double* param, char param_no) // "tx1_attenuation?" command
{
	uint32_t attenuation_mdb;

	ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb);
	console_print("tx1_attenuation=%d\n", attenuation_mdb);
}

/**************************************************************************//***
 * @brief Sets the TX1 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void set_tx1_attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb;

	if(param_no >= 1)
	{
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 0, attenuation_mdb);
		console_print("tx1_attenuation=%d\n", attenuation_mdb);
		/* MOD-26b: remember it as the operating level for every restart */
		if(attenuation_mdb < GNSS_L1_TX_MAX_ATTEN_MDB) {
			gnss_user_tx1_mdb   = attenuation_mdb;
			gnss_plan_atten_mdb = attenuation_mdb;
			gnss_op_atten_mdb   = attenuation_mdb;
			console_print("  (kept as the TX1 operating level: gnss_band= / gnss_auto=1 reopen to it)\n");
		}
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX2 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void get_tx2_attenuation(double* param, char param_no) // "tx1_attenuation?" command
{
	uint32_t attenuation_mdb;

	ad9361_get_tx_attenuation(ad9361_phy, 1, &attenuation_mdb);
	console_print("tx2_attenuation=%d\n", attenuation_mdb);
}

/**************************************************************************//***
 * @brief Sets the TX2 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void set_tx2_attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb;

	if(param_no >= 1)
	{
		/* MOD-26: TX2 is unused and held quiet (gnss_config.h) */
		attenuation_mdb = param[0];
		if(attenuation_mdb != GNSS_CFG_TX2_ATTEN_MDB)
			console_print("tx2_attenuation: TX2 is unused and held at %d mdB "
				      "(GNSS_CFG_TX2_ATTEN_MDB); %d not applied.\n",
				      (long)GNSS_CFG_TX2_ATTEN_MDB, (long)attenuation_mdb);
		console_print("tx2_attenuation=%d\n", (long)gnss_tx2_hold());
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX FIR state.
 *
 * @return None.
*******************************************************************************/
void get_tx_fir_en(double* param, char param_no) // "tx_fir_en?" command
{
	uint8_t en_dis;

	ad9361_get_tx_fir_en_dis(ad9361_phy, &en_dis);
	console_print("tx_fir_en=%d\n", en_dis);
}

/**************************************************************************//***
 * @brief Sets the TX FIR state.
 *
 * @return None.
*******************************************************************************/
void set_tx_fir_en(double* param, char param_no) // "tx_fir_en=" command
{
	uint8_t en_dis;

	if(param_no >= 1)
	{
		en_dis = param[0];
		ad9361_set_tx_fir_en_dis(ad9361_phy, en_dis);
		console_print("tx_fir_en=%d\n", en_dis);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void get_rx_lo_freq(double* param, char param_no) // "rx_lo_freq?" command
{
	uint64_t lo_freq_hz;

	ad9361_get_rx_lo_freq(ad9361_phy, &lo_freq_hz);
	/* Exact Hz, not truncated MHz -- see the comment in get_tx_lo_freq(). */
	console_print("rx_lo_freq=%d\n", (long)lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void set_rx_lo_freq(double* param, char param_no) // "rx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no >= 1)
	{
		/* BUG FIX -- same issue and same fix as set_tx_lo_freq() above: scale
		 * to Hz in floating point before truncating to uint64_t, or a
		 * fractional-MHz request like rx_lo_freq=1575.42 silently loses the
		 * .42 MHz. Confirmed on hardware via gnss_mvdr=1's radio defaults. */
		lo_freq_hz = (uint64_t)(param[0] * 1000000.0 + 0.5);   /* MOD-15: round, never truncate */
#ifdef ANTSDR_E310
		/* set rx rf swicth */
		if(lo_freq_hz <= 3000000000){
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx1_ctrl_h   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx1_ctrl_l   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx2_ctrl_h   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx2_ctrl_l   ,1);
			ad9361_set_rx_rf_port_input(ad9361_phy, B_BALANCED);
		}
		else {
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx1_ctrl_h   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx1_ctrl_l   ,0);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx2_ctrl_h   ,1);
			no_os_gpio_set_value(ad9361_phy->gpio_desc_rx2_ctrl_l   ,0);
			ad9361_set_rx_rf_port_input(ad9361_phy, A_BALANCED);
		}

#else
		ad9361_set_rx_rf_port_input(ad9361_phy, A_BALANCED);
#endif
		ad9361_set_rx_lo_freq(ad9361_phy, lo_freq_hz);
		/* Exact Hz, not truncated MHz -- see the comment in get_tx_lo_freq(). */
		console_print("rx_lo_freq=%d\n", (long)lo_freq_hz);
	}
}

/**************************************************************************//***
 * @brief Gets current RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_rx_samp_freq(double* param, char param_no) // "rx_samp_freq?" command
{
	uint32_t sampling_freq_hz;

	ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("rx_samp_freq=%d\n", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_rx_samp_freq(double* param, char param_no) // "rx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no >= 1)
	{
		sampling_freq_hz = (uint32_t)param[0];
		ad9361_set_rx_sampling_freq(ad9361_phy, sampling_freq_hz);
		ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
//		dds_update(ad9361_phy);
		console_print("rx_samp_freq=%d\n", sampling_freq_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void get_rx_rf_bandwidth(double* param, char param_no) // "rx_rf_bandwidth?" command
{
	uint32_t bandwidth_hz;

	ad9361_get_rx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("rx_rf_bandwidth=%d\n", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void set_rx_rf_bandwidth(double* param, char param_no) // "rx_rf_bandwidth=" command
{
	uint32_t bandwidth_hz;

	if(param_no >= 1)
	{
		bandwidth_hz = param[0];
		ad9361_set_rx_rf_bandwidth(ad9361_phy, bandwidth_hz);
		console_print("rx_rf_bandwidth=%d\n", bandwidth_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX1 GC mode.
 *
 * @return None.
*******************************************************************************/
void get_rx1_gc_mode(double* param, char param_no) // "rx1_gc_mode?" command
{
	uint8_t gc_mode;

	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &gc_mode);
	console_print("rx1_gc_mode=%d\n", gc_mode);
}

/**************************************************************************//***
 * @brief Sets the RX1 GC mode.
 *
 * @return None.
*******************************************************************************/
void set_rx1_gc_mode(double* param, char param_no) // "rx1_gc_mode=" command
{
	uint8_t gc_mode;

	if(param_no >= 1)
	{
		gc_mode = param[0];
		ad9361_set_rx_gain_control_mode(ad9361_phy, 0, gc_mode);
		console_print("rx1_gc_mode=%d\n", gc_mode);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX2 GC mode.
 *
 * @return None.
*******************************************************************************/
void get_rx2_gc_mode(double* param, char param_no) // "rx2_gc_mode?" command
{
	uint8_t gc_mode;

	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &gc_mode);
	console_print("rx2_gc_mode=%d\n", gc_mode);
}

/**************************************************************************//***
 * @brief Sets the RX2 GC mode.
 *
 * @return None.
*******************************************************************************/
void set_rx2_gc_mode(double* param, char param_no) // "rx2_gc_mode=" command
{
	uint8_t gc_mode;

	if(param_no >= 1)
	{
		gc_mode = param[0];
		ad9361_set_rx_gain_control_mode(ad9361_phy, 1, gc_mode);
		console_print("rx2_gc_mode=%d\n", gc_mode);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX1 RF gain.
 *
 * @return None.
*******************************************************************************/
void get_rx1_rf_gain(double* param, char param_no) // "rx1_rf_gain?" command
{
	int32_t gain_db;

	ad9361_get_rx_rf_gain (ad9361_phy, 0, &gain_db);
	console_print("rx1_rf_gain=%d\n", gain_db);
}

/**************************************************************************//***
 * @brief Sets the RX1 RF gain.
 *
 * @return None.
*******************************************************************************/
void set_rx1_rf_gain(double* param, char param_no) // "rx1_rf_gain=" command
{
	int32_t gain_db;

	if(param_no >= 1)
	{
		gain_db = param[0];
		ad9361_set_rx_rf_gain (ad9361_phy, 0, gain_db);
		console_print("rx1_rf_gain=%d\n", gain_db);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX2 RF gain.
 *
 * @return None.
*******************************************************************************/
void get_rx2_rf_gain(double* param, char param_no) // "rx2_rf_gain?" command
{
	int32_t gain_db;

	ad9361_get_rx_rf_gain (ad9361_phy, 1, &gain_db);
	console_print("rx2_rf_gain=%d\n", gain_db);
}

/**************************************************************************//***
 * @brief Sets the RX2 RF gain.
 *
 * @return None.
*******************************************************************************/
void set_rx2_rf_gain(double* param, char param_no) // "rx2_rf_gain=" command
{
	int32_t gain_db;

	if(param_no >= 1)
	{
		gain_db = param[0];
		ad9361_set_rx_rf_gain (ad9361_phy, 1, gain_db);
		console_print("rx2_rf_gain=%d\n", gain_db);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX FIR state.
 *
 * @return None.
*******************************************************************************/
void get_rx_fir_en(double* param, char param_no) // "rx_fir_en?" command
{
	uint8_t en_dis;

	ad9361_get_rx_fir_en_dis(ad9361_phy, &en_dis);
	console_print("rx_fir_en=%d\n", en_dis);
}

/**************************************************************************//***
 * @brief Sets the RX FIR state.
 *
 * @return None.
*******************************************************************************/
void set_rx_fir_en(double* param, char param_no) // "rx_fir_en=" command
{
	uint8_t en_dis;

	if(param_no >= 1)
	{
		en_dis = param[0];
		ad9361_set_rx_fir_en_dis(ad9361_phy, en_dis);
		console_print("rx_fir_en=%d\n", en_dis);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_freq(double* param, char param_no)	// dds_tx1_tone1_freq?
{
	uint32_t freq;
	axi_dac_dds_get_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, &freq);
	console_print("dds_tx1_tone1_freq=%d\n", freq);
}
/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_freq(double* param, char param_no)	// dds_tx1_tone1_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, freq);
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F1, freq);
		console_print("dds_tx1_tone1_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_freq(double* param, char param_no)	// dds_tx1_tone2_freq?
{
	uint32_t freq ;
	axi_dac_dds_get_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, &freq);
	console_print("dds_tx1_tone2_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_freq(double* param, char param_no)	// dds_tx1_tone2_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, freq);
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F2, freq);
		console_print("dds_tx1_tone2_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_phase(double* param, char param_no)	// dds_tx1_tone1_phase?
{
	uint32_t phase ;
	axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, &phase);
	phase /= 1000;
	console_print("dds_tx1_tone1_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_phase(double* param, char param_no)	// dds_tx1_tone1_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F1, (uint32_t)((phase - 90) * 1000));
		axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, &phase);
		phase /= 1000;
		console_print("dds_tx1_tone1_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_phase(double* param, char param_no)	// dds_tx1_tone2_phase?
{
	uint32_t phase;
	axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, &phase);
	phase /= 1000;
	console_print("dds_tx1_tone2_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_phase(double* param, char param_no)	// dds_tx1_tone2_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F2, (uint32_t)((phase - 90) * 1000));
		axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, &phase);
		phase /= 1000;
		console_print("dds_tx1_tone2_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_scale(double* param, char param_no)	// dds_tx1_tone1_scale?
{
	int32_t scale ;
	axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, &scale);
	console_print("dds_tx1_tone1_scale=%d\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_scale(double* param, char param_no)	// dds_tx1_tone1_scale=
{
	int32_t scale = (int32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, scale);
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F1, scale);
		axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F1, &scale);
		console_print("dds_tx1_tone1_scale=%d\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_scale(double* param, char param_no)	// dds_tx1_tone2_scale?
{
	int32_t scale;
	axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, &scale);
	console_print("dds_tx1_tone2_scale=%d\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_scale(double* param, char param_no)	// dds_tx1_tone2_scale=
{
	int32_t scale = (int32_t)param[0];

	if(param_no >= 1)
	{

		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, scale);
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_Q_F2, scale);
		axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX1_I_F2, &scale);
		console_print("dds_tx1_tone2_scale=%d\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_freq(double* param, char param_no)	// dds_tx2_tone1_freq?
{
	uint32_t freq;
	axi_dac_dds_get_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, &freq);
	console_print("dds_tx2_tone1_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_freq(double* param, char param_no)	// dds_tx2_tone1_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, freq);
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F1, freq);
		console_print("dds_tx2_tone1_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone2_freq(double* param, char param_no)	// dds_tx2_tone2_freq?
{
	uint32_t freq;
	axi_dac_dds_get_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, &freq);
	console_print("dds_tx2_tone2_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_freq(double* param, char param_no)	// dds_tx2_tone2_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, freq);
		axi_dac_dds_set_frequency(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F2, freq);
		console_print("dds_tx2_tone2_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_phase(double* param, char param_no)	// dds_tx2_tone1_phase?
{
	uint32_t phase;
	axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, &phase);
	phase /= 1000;
	console_print("dds_tx2_tone1_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_phase(double* param, char param_no)	// dds_tx2_tone1_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F1, (uint32_t)((phase - 90) * 1000));

		axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, &phase);
		phase /= 1000;
		console_print("dds_tx2_tone1_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone2_phase(double* param, char param_no)	// dds_tx2_tone2_phase?
{
	uint32_t phase ;
	axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, &phase);
	phase /= 1000;
	console_print("dds_tx2_f2_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_phase(double* param, char param_no)	// dds_tx2_tone2_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		axi_dac_dds_set_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F2, (uint32_t)((phase - 90) * 1000));
		axi_dac_dds_get_phase(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, &phase);
		phase /= 1000;
		console_print("dds_tx2_tone2_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_scale(double* param, char param_no)	// dds_tx2_tone1_scale?
{
	int32_t scale ;
	axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, &scale);
	console_print("dds_tx2_tone1_scale=%d\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_scale(double* param, char param_no)	// dds_tx2_tone1_scale=
{
	int32_t scale = (int32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, scale);
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F1, scale);
		axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F1, &scale);
		console_print("dds_tx2_tone1_scale=%d\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void dds_tx2_tone2_scale(double* param, char param_no)	// dds_tx2_tone2_scale?
{
	int32_t scale;
	axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, &scale);

	console_print("dds_tx2_tone2_scale=%d\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_scale(double* param, char param_no)	// dds_tx2_tone2_scale=
{
	int32_t scale = (int32_t)param[0];

	if(param_no >= 1)
	{
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, scale);
		axi_dac_dds_set_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_Q_F2, scale);
		axi_dac_dds_get_scale(ad9361_phy->tx_dac, DDS_CHAN_TX2_I_F2, &scale);
		console_print("dds_tx2_tone2_scale=%d\n", scale);
	}
	else
		show_invalid_param_message(1);
}
