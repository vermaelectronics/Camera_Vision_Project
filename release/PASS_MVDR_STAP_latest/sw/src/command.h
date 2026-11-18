/**************************************************************************//**
 *   @file   command.h
 *   @brief  Header file of AD9361 Command Driver.
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
#ifndef __COMMAND_H__
#define __COMMAND_H__
#include <stdint.h>   /* MOD-15: uint32_t in the prototypes below */

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define NULL		((void *)0)
#define SUCCESS		0
#define ERROR		-1

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
typedef void (*cmd_function)(double* param, char param_no);
typedef struct
{
	const char* name;
	const char* description;
	const char* example;
	cmd_function function;
}command;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Displays all available commands. */
void get_help(double* param, char param_no);

/* Gets the specified register value. */
void get_register(double* param, char param_no);

/* Gets current TX LO frequency. */
void get_tx_lo_freq(double* param, char param_no);

/* Sets the TX LO frequency. */
void set_tx_lo_freq(double* param, char param_no);

/* Gets current TX sampling frequency. */
void get_tx_samp_freq(double* param, char param_no);

/* Sets the TX sampling frequency. */
void set_tx_samp_freq(double* param, char param_no);

/* Gets current TX RF bandwidth. */
void get_tx_rf_bandwidth(double* param, char param_no);

/* Sets the TX RF bandwidth. */
void set_tx_rf_bandwidth(double* param, char param_no);

/* Gets current TX1 attenuation. */
void get_tx1_attenuation(double* param, char param_no);

/* Sets the TX1 attenuation. */
void set_tx1_attenuation(double* param, char param_no);

/* Gets current TX2 attenuation. */
void get_tx2_attenuation(double* param, char param_no);

/* Sets the TX2 attenuation. */
void set_tx2_attenuation(double* param, char param_no);

/* Gets current TX FIR state. */
void get_tx_fir_en(double* param, char param_no);

/* Sets the TX FIR state. */
void set_tx_fir_en(double* param, char param_no);

/* Gets current RX LO frequency. */
void get_rx_lo_freq(double* param, char param_no);

/* Sets the RX LO frequency. */
void set_rx_lo_freq(double* param, char param_no);

/* Gets current RX sampling frequency. */
void get_rx_samp_freq(double* param, char param_no);

/* Sets the RX sampling frequency. */
void set_rx_samp_freq(double* param, char param_no);

/* Gets current RX RF bandwidth. */
void get_rx_rf_bandwidth(double* param, char param_no);

/* Sets the RX RF bandwidth. */
void set_rx_rf_bandwidth(double* param, char param_no);

/* Gets current RX1 GC mode. */
void get_rx1_gc_mode(double* param, char param_no);

/* Sets the RX1 GC mode. */
void set_rx1_gc_mode(double* param, char param_no);

/* Gets current RX2 GC mode. */
void get_rx2_gc_mode(double* param, char param_no);

/* Sets the RX2 GC mode. */
void set_rx2_gc_mode(double* param, char param_no);

/* Gets current RX1 RF gain. */
void get_rx1_rf_gain(double* param, char param_no);

/* Sets the RX1 RF gain. */
void set_rx1_rf_gain(double* param, char param_no);

/* Gets current RX2 RF gain. */
void get_rx2_rf_gain(double* param, char param_no);

/* Sets the RX2 RF gain. */
void set_rx2_rf_gain(double* param, char param_no);

/* Gets current RX FIR state. */
void get_rx_fir_en(double* param, char param_no);

/* Sets the RX FIR state. */
void set_rx_fir_en(double* param, char param_no);

/* Gets current DDS TX1 Tone 1 frequency [Hz]. */
void get_dds_tx1_tone1_freq(double* param, char param_no);

/* Sets the DDS TX1 Tone 1 frequency [Hz]. */
void set_dds_tx1_tone1_freq(double* param, char param_no);

/* Gets current DDS TX1 Tone 2 frequency [Hz]. */
void get_dds_tx1_tone2_freq(double* param, char param_no);

/* Sets the DDS TX1 Tone 2 frequency [Hz]. */
void set_dds_tx1_tone2_freq(double* param, char param_no);

/* Gets current DDS TX1 Tone 1 phase [degrees]. */
void get_dds_tx1_tone1_phase(double* param, char param_no);

/* Sets the DDS TX1 Tone 1 phase [degrees]. */
void set_dds_tx1_tone1_phase(double* param, char param_no);

/* Gets current DDS TX1 Tone 2 phase [degrees]. */
void get_dds_tx1_tone2_phase(double* param, char param_no);

/* Sets the DDS TX1 Tone 2 phase [degrees]. */
void set_dds_tx1_tone2_phase(double* param, char param_no);

/* Gets current DDS TX1 Tone 1 scale. */
void get_dds_tx1_tone1_scale(double* param, char param_no);

/* Sets the DDS TX1 Tone 1 scale. */
void set_dds_tx1_tone1_scale(double* param, char param_no);

/* Gets current DDS TX1 Tone 2 scale. */
void get_dds_tx1_tone2_scale(double* param, char param_no);

/* Sets the DDS TX1 Tone 2 scale. */
void set_dds_tx1_tone2_scale(double* param, char param_no);

/* Gets current DDS TX2 Tone 1 frequency [Hz]. */
void get_dds_tx2_tone1_freq(double* param, char param_no);

/* Sets the DDS TX2 Tone 1 frequency [Hz]. */
void set_dds_tx2_tone1_freq(double* param, char param_no);

/* Gets current DDS TX2 Tone 2 frequency [Hz]. */
void get_dds_tx2_tone2_freq(double* param, char param_no);

/* Sets the DDS TX2 Tone 2 frequency [Hz]. */
void set_dds_tx2_tone2_freq(double* param, char param_no);

/* Gets current DDS TX2 Tone 1 phase [degrees]. */
void get_dds_tx2_tone1_phase(double* param, char param_no);

/* Sets the DDS TX2 Tone 1 phase [degrees]. */
void set_dds_tx2_tone1_phase(double* param, char param_no);

/* Gets current DDS TX2 Tone 2 phase [degrees]. */
void get_dds_tx2_tone2_phase(double* param, char param_no);

/* Sets the DDS TX2 Tone 2 phase [degrees]. */
void set_dds_tx2_tone2_phase(double* param, char param_no);

/* Gets current DDS TX2 Tone 1 scale. */
void get_dds_tx2_tone1_scale(double* param, char param_no);

/* Sets the DDS TX2 Tone 1 scale. */
void set_dds_tx2_tone1_scale(double* param, char param_no);

/* Gets current DDS TX2 Tone 2 scale. */
void dds_tx2_tone2_scale(double* param, char param_no);

/* Sets the DDS TX2 Tone 2 scale. */
void set_dds_tx2_tone2_scale(double* param, char param_no);

/* GNSS-CRPA MOD-6: live console control of the L1 retransmit path.
 * See Docs/Architecture/MODIFICATIONS.md. */

/* Gets the GNSS L1 retransmit state, read back from hardware. */
void get_gnss_tx(double* param, char param_no);

/* 1 = retransmit RX1 on TX1 at MAXIMUM attenuation, 0 = silence TX1.
 * gnss_tx=0 is the abort path and is safe to issue at any time. */
void set_gnss_tx(double* param, char param_no);

/* Prints AD9361 and gnss_passthrough runtime state. */
void get_gnss_status(double* param, char param_no);

/* GNSS-CRPA MOD-9: the main menu, bound to "?".
 * Short and grouped, unlike get_help() which dumps all ~60 vendor commands. */
void get_menu(double* param, char param_no);
/* LVDS / ant board: the MVDR2 gnss_tx=1/0 (raw RX1 retransmit / abort) and its
 * hardware read-back, now used by pass= / gnss_tx= in gnss_ant.c */
void gnss_tx_raw(double* param, char param_no);
void gnss_tx_hw_print(double* param, char param_no);
int  gnss_radio_defaults_ready(void);
uint32_t gnss_get_op_atten_mdb(void);

/* GNSS-CRPA MOD-7: the DDR round trip.
 * RX -> DMA -> DDR -> DMA -> axi_ad9361 -> TX1 (pass_en = 0 branch). */

/* Gets the DDR round-trip replay state. */
void get_gnss_ddr_tx(double* param, char param_no);

/* 1 = capture RX to DDR and replay it cyclically on TX1, 0 = stop.
 * gnss_ddr_tx=0 is the abort path and is safe to issue at any time. */
void set_gnss_ddr_tx(double* param, char param_no);

/* GNSS-CRPA MOD-11: live console control of the adaptive 2-channel MVDR
 * beamformer. Independent of gnss_tx=: gnss_mvdr=0 falls back to the same
 * raw RX1 retransmit gnss_tx= drives, it does NOT stop TX1.
 * See Docs/Architecture/MODIFICATIONS.md. */

/* Prints MVDR mode, steering vector, current weights (magnitude/phase),
 * covariance sums, update/skip/saturation counters, and a plain-English
 * verdict on whether it is actually nulling anything right now. */
void get_gnss_mvdr(double* param, char param_no);

/* 1 = make sure the shared radio plan is in (applied ONCE, values in
 * gnss_config.h: LO 1575.42 MHz, 40 MHz BW, 45 MSPS, RX1/RX2 MANUAL, TX1
 * 64000 mdB -- never re-applied,
 * so gnss_rxgain= / tx1_attenuation= values survive), enable adaptive MVDR
 * with power-inversion steering, jammer averaging 2^6 windows, and route it
 * to TX1 at the operating attenuation. Warns if RX1/RX2 gain is not manual
 * and equal.
 * 0 = ch0 reverts to raw RX1; TX1 keeps transmitting whatever gnss_tx= last
 * set -- gnss_mvdr=0 does NOT silence TX1. */
void set_gnss_mvdr(double* param, char param_no);

/* GNSS-CRPA MOD-12: jammer estimator (jammer_est.v in the MVDR window).
 * Names are chosen so that none is a prefix of another (the console matches
 * on the name up to '?' / '='): gnss_jam?, gnss_jmon=, gnss_jcal=,
 * gnss_jthr=, gnss_jinv=. */

/* Prints jammer present / "No jammer present", angle, power, JNR, phase. */
void get_gnss_jam(double* param, char param_no);

/* gnss_jmon=N : N one-line readings, one per second (1..600). */
void set_gnss_jmon(double* param, char param_no);

/* gnss_jcal=1 : calibrate RX1/RX2 phase with a source at broadside.
 * gnss_jcal=0 : clear the calibration. */
void set_gnss_jcal(double* param, char param_no);

/* gnss_jthr=dB : detection threshold (release is 2 dB lower), 1..40. */
void set_gnss_jthr(double* param, char param_no);

/* gnss_jinv=0/1 : invert the phase sign if the angle moves the wrong way. */
void set_gnss_jinv(double* param, char param_no);

/* gnss_jpcal=P : dBm calibration with a CW generator of P dBm at the RX input.
 * A positive number is taken as negative (gnss_jpcal=60 means -60 dBm). */
void set_gnss_jpcal(double* param, char param_no);

/* GNSS-CRPA MOD-13: input level and output gain for MVDR v1.4.
 * gnss_rxgain / gnss_ogain are not a prefix of any other name, nor the reverse. */

/* gnss_rxgain? : RX1/RX2 gain + mode, input level and noise floor in dBFS,
 * ADC clip counts / headroom, and the dB change to the noise target in use
 * (-60 dBFS, or higher when the AD9361 floor is high; gnss_rxauto=1). */
void get_gnss_rxgain(double* param, char param_no);

/* gnss_rxgain=N : RX1 AND RX2 to manual gain, both N dB (0..71). */
void set_gnss_rxgain(double* param, char param_no);

/* gnss_ogain? : OUT_GAIN, output clip counter, DAC level, suggested n. */
void get_gnss_ogain(double* param, char param_no);

/* gnss_ogain=n : MVDR output gain 2^n, n = 0..7 (+6.02 dB per step). */
void set_gnss_ogain(double* param, char param_no);

/* Applies the shared radio plan (LO/BW/rate, RX1/RX2 manual gain, operating
 * TX1 attenuation value). Call ONCE from main.c right after the AD9361 is
 * initialised so the board boots with it; otherwise the first gnss_tx=1 /
 * gnss_mvdr=1 / gnss_rxgain= applies it. Does not open any TX path. */
void gnss_radio_apply_defaults(void);

/* GNSS-CRPA MOD-14: MVDR block v1.12 features. Names are not prefixes of each
 * other or of any other command. */

/* gnss_aolc? : output level control state, measured level and gain. */
void get_gnss_aolc(double* param, char param_no);

/* gnss_aolc=T : automatic output level at -T dBFS (6..40, 18 recommended);
 * 0 = off (manual gnss_ogain=). Raises TX1: attenuation to max meanwhile. */
void set_gnss_aolc(double* param, char param_no);

/* gnss_iq? : receiver IQ-image correction coefficients (dBc), updates. */
void get_gnss_iq(double* param, char param_no);

/* gnss_iq=1/0/2 : IQ-image correction on / off / clear and restart. */
void set_gnss_iq(double* param, char param_no);

/* gnss_coh=N : MVDR jammer detect threshold, coherence > 2^-N (1..15). */
void set_gnss_coh(double* param, char param_no);
/* GNSS-CRPA MOD-21: gnss_null? / gnss_null=0|1 (MVDR / plain power inversion) */
void get_gnss_null(double* param, char param_no);
void set_gnss_null(double* param, char param_no);
/* GNSS-CRPA MOD-24: gnss_null=2 STAP, gnss_supp? suppression report,
 * gnss_stap? / gnss_stap=mu */
void get_gnss_supp(double* param, char param_no);
void get_gnss_stap(double* param, char param_no);
void set_gnss_stap(double* param, char param_no);
/* GNSS-CRPA MOD-25: gnss_null=3 SMI-MVDR STAP, gnss_smi?/=, gnss_steer?/=,
 * gnss_lookph= */
void get_gnss_smi(double* param, char param_no);
void set_gnss_smi(double* param, char param_no);
void get_gnss_steer(double* param, char param_no);
void set_gnss_steer(double* param, char param_no);
void set_gnss_lookph(double* param, char param_no);
/* GNSS-CRPA MOD-25: gnss_check? / gnss_cmds? / gnss_color= */
void get_gnss_check(double* param, char param_no);
void get_gnss_cmds(double* param, char param_no);
void get_gnss_report(double* param, char param_no);  /* MOD-26 gnss_report? */
void get_gnss_beam(double* param, char param_no);    /* MOD-26 gnss_beam?    */
void get_gnss_floor(double* param, char param_no);   /* MOD-27 gnss_floor?   */
void get_gnss_kpi(double* param, char param_no);     /* MOD-28 gnss_kpi?     */
void get_gnss_spec(double* param, char param_no);    /* MOD-29 gnss_spec?    */
void set_gnss_spec(double* param, char param_no);    /* MOD-29 gnss_spec=    */
void get_gnss_bist(double* param, char param_no);    /* MOD-29 gnss_bist?    */
void get_gnss_clk(double* param, char param_no);     /* MOD-30 gnss_clk?     */
void set_gnss_osr(double* param, char param_no);     /* MOD-30 gnss_osr=     */
void set_gnss_floor(double* param, char param_no);   /* MOD-30 gnss_floor=   */
void set_gnss_smiavg(double* param, char param_no);  /* MOD-31 gnss_smiavg=  */
void gnss_clk_print(void);                            /* MOD-30 */
void gnss_osr_apply(void);                            /* MOD-30 */
int  gnss_adc_osr_now(void);                          /* MOD-30: ADC / fs, 0 unknown */
void set_gnss_beam(double* param, char param_no);    /* MOD-26 gnss_beam=    */
void get_gnss_mvsteer(double* param, char param_no); /* MOD-26 gnss_mvsteer? */
void set_gnss_mvsteer(double* param, char param_no); /* MOD-26 gnss_mvsteer= */
void gnss_band_info(int *n, const char **name);       /* MOD-26 */
int  gnss_tx_is_on(void);                             /* MOD-26 */
uint32_t gnss_tx2_hold(void);                         /* MOD-26: TX2 held quiet */
void set_gnss_color(double* param, char param_no);

/* GNSS-CRPA MOD-15: automatic start / health / RX auto-level. */
void set_gnss_auto(double* param, char param_no);    /* gnss_auto=1 / =0   */
void get_gnss_health(double* param, char param_no);  /* gnss_health?       */
void set_gnss_rxauto(double* param, char param_no);  /* gnss_rxauto=1      */
void get_gnss_guard(double* param, char param_no);   /* gnss_guard?        */
void set_gnss_guard(double* param, char param_no);   /* gnss_guard=1 / =0  */
/* GNSS-CRPA MOD-16 */
void get_gnss_lo(double* param, char param_no);      /* gnss_lo?           */
void set_gnss_lo(double* param, char param_no);      /* gnss_lo=MHz        */
uint64_t gnss_get_lo_hz(void);                       /* LO in use          */
/* GNSS-CRPA MOD-17: runtime radio plan / presets / RX gain mode */
void gnss_get_plan(uint64_t *lo, uint32_t *bw, uint32_t *fs, int32_t *gain0, uint32_t *atten);
int  gnss_select_band(int n);
void get_gnss_band(double* param, char param_no);    /* gnss_band?         */
void set_gnss_band(double* param, char param_no);    /* gnss_band=N        */
void set_gnss_plan(double* param, char param_no);    /* gnss_plan=LO,BW,FS */
void get_gnss_agc(double* param, char param_no);     /* gnss_agc?          */
void set_gnss_agc(double* param, char param_no);     /* gnss_agc=1/0       */
void gnss_set_tx1_attenuation(double* param, char param_no);
/* GNSS-CRPA MOD-20: MVDR-aware LO / rate / bandwidth (RX+TX, restart MVDR) */
void gnss_set_rx_lo_freq(double* p, char n);
void gnss_set_tx_lo_freq(double* p, char n);
void gnss_set_rx_samp_freq(double* p, char n);
void gnss_set_tx_samp_freq(double* p, char n);
void gnss_set_rx_rf_bandwidth(double* p, char n);
void gnss_set_tx_rf_bandwidth(double* p, char n);
void gnss_set_rx1_gc_mode(double* param, char param_no);  /* MVDR-aware   */
void gnss_set_rx2_gc_mode(double* param, char param_no);  /* vendor rx    */
void gnss_set_rx1_rf_gain(double* param, char param_no);  /* gain / mode  */
void gnss_set_rx2_rf_gain(double* param, char param_no);  /* commands     */

/* GNSS-CRPA MOD-15: TX1 operating attenuation (mdB) that gnss_tx=1 /
 * gnss_mvdr=1 open up to as their last step. */
void gnss_tx_silence_quiet(void);   /* MOD-19: silence without the OFF message */
void gnss_tx_mark_on(int mvdr);       /* MOD-19: record that TX1 was opened      */
uint32_t gnss_get_op_atten_mdb(void);
void     gnss_set_op_atten_mdb(uint32_t mdb);

#endif  // __COMMAND_H__
