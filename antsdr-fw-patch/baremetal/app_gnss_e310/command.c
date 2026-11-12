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
#include "gnss_log.h"
#include "console.h"
#include "ad9361_api.h"
#include "axi_dac_core.h"
//#include "platform.h"
#include "parameters.h"
#include "app_config.h"
/* GNSS-CRPA MOD-6 */
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_smi_agc.h"
#include "gnss_aj.h"
#include "gnss_aj_math.h"
#include "gnss_notch.h"
/* GNSS-CRPA MOD-7 */
#include "gnss_txdma.h"
#include "axi_dmac.h"
/* GNSS-CRPA MOD-10 */
#include "gnss_info.h"
#include "gnss_vcxo.h"     /* antsdr-fw-patch: design-A 40 MHz VCXO */

/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
command cmd_list[] = {
	/* GNSS-CRPA MOD-9: "?" on its own is the main menu.
	 *
	 * This works because console_check_commands() walks the EXPECTED name only
	 * until the first '!', '?' or '='. For the name "?" that is index 0, so no
	 * characters are compared and the match reduces to
	 * (received_cmd[0] == '?'). Nothing else in the table starts with '?', so
	 * there is no collision -- and a real command like "tx_lo_freq?" is never
	 * matched by it, because its first character is not '?'. */
	{"?", "This menu. Type help? for every command.", "?", get_menu},
	{"help?", "Displays all available commands.", "", get_help},
	{"register?", "Gets the specified register value.", "", get_register},
	{"tx_lo_freq?", "Gets current TX LO frequency [MHz].", "", get_tx_lo_freq},
	{"tx_lo_freq=", "Sets the TX LO frequency [MHz].", "", set_tx_lo_freq},
	{"tx_samp_freq?", "Gets current TX sampling frequency [Hz].", "", get_tx_samp_freq},
	{"tx_samp_freq=", "Sets the TX sampling frequency [Hz].", "", set_tx_samp_freq},
	{"tx_rf_bandwidth?", "Gets current TX RF bandwidth [Hz].", "", get_tx_rf_bandwidth},
	{"tx_rf_bandwidth=", "Sets the TX RF bandwidth [Hz].", "", set_tx_rf_bandwidth},
	{"tx1_attenuation?", "Gets current TX1 attenuation [mdB].", "", get_tx1_attenuation},
	{"tx1_attenuation=", "Sets the TX1 attenuation [mdB].", "", set_tx1_attenuation},
	{"tx_att_db=", "TX1 attenuation in dB, 0 to 89.75 in 0.25 steps (smaller = louder).", "tx_att_db=70.5", set_tx_att_db},
	{"tx2_attenuation?", "Gets current TX2 attenuation [mdB].", "", get_tx2_attenuation},
	{"tx2_attenuation=", "Sets the TX2 attenuation [mdB].", "", set_tx2_attenuation},
	{"gnss_tx2?", "TX2 state: muted (default) or carrying a copy of the CRPA output.", "", get_gnss_tx2},
	{"gnss_tx2=", "0 = TX2 muted (default), 1 = TX2 carries a copy of the CRPA output (TX1 samples), at 89.75 dB.", "gnss_tx2=0", set_gnss_tx2},
	{"tx_fir_en?", "Gets current TX FIR state.", "", get_tx_fir_en},
	{"tx_fir_en=", "Sets the TX FIR state.", "", set_tx_fir_en},
	{"rx_lo_freq?", "Gets current RX LO frequency [MHz].", "", get_rx_lo_freq},
	{"rx_lo_freq=", "Sets the RX LO frequency [MHz].", "", set_rx_lo_freq},
	{"rx_samp_freq?", "Gets current RX sampling frequency [Hz].", "", get_rx_samp_freq},
	{"rx_samp_freq=", "Sets the RX sampling frequency [Hz].", "", set_rx_samp_freq},
	{"rx_rf_bandwidth?", "Gets current RX RF bandwidth [Hz].", "", get_rx_rf_bandwidth},
	{"rx_rf_bandwidth=", "Sets the RX RF bandwidth [Hz].", "", set_rx_rf_bandwidth},
	{"rx1_gc_mode?", "Gets current RX1 GC mode.", "", get_rx1_gc_mode},
	{"rx1_gc_mode=", "Sets the RX1 GC mode.", "", set_rx1_gc_mode},
	{"rx2_gc_mode?", "Gets current RX2 GC mode.", "", get_rx2_gc_mode},
	{"rx2_gc_mode=", "Sets the RX2 GC mode.", "", set_rx2_gc_mode},
	{"rx1_rf_gain?", "Gets current RX1 RF gain.", "", get_rx1_rf_gain},
	{"rx1_rf_gain=", "Sets the RX1 RF gain.", "", set_rx1_rf_gain},
	{"rx2_rf_gain?", "Gets current RX2 RF gain.", "", get_rx2_rf_gain},
	{"rx2_rf_gain=", "Sets the RX2 RF gain.", "", set_rx2_rf_gain},
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
	/* GNSS-CRPA MOD-6: live control of the RX->PL->TX retransmit path, so a
	 * conducted test can be started, stepped and ABORTED from the console
	 * without reprogramming the board. See Docs/Architecture/MODIFICATIONS.md. */
	{"radio?", "Shows RX/TX LO, sample rate, bandwidth, RX gain, TX attenuation.", "", get_radio},
	{"radio=", "Sets everything by hand: radio=<rxLO> <txLO> <rxRate> <txRate> <txAtt dB> <rxGain dB> [bw]  or  radio=<LO> <rate> <txAtt> <rxGain> [bw].", "radio=1575.42 1575.42 30.72 30.72 70 40 18", set_radio},
	{"gnss_tx?", "Gets the GNSS L1 retransmit state.", "", get_gnss_tx},
	{"gnss_tx=", "1 = retransmit RX1 on TX1, 0 = silence TX1.", "gnss_tx=0", set_gnss_tx},
	{"gnss_status?", "Prints AD9361 and gnss_passthrough runtime state.", "", get_gnss_status},
	/* SMI-PI v4.1 space-time anti-jam (smi_pi_0 v2.1 + gnss_aj.c). */
	{"gnss_smi?", "Anti-jam state: on/off, jammer, J/N, suppression.", "", get_gnss_smi},
	{"gnss_smi=", "1 = anti-jam ON (default at boot), 0 = OFF (TX1 carries RX1).", "gnss_smi=1", set_gnss_smi},
	{"gnss_smi_freeze=", "1 = hold the filter exactly as it is (weights, bypass), 0 = automatic (default).", "gnss_smi_freeze=0", set_gnss_smi_freeze},
	{"gnss_aj?", "Anti-jam status: bypass/filter, jammer type and frequency, J/N, suppression.", "", get_gnss_aj},
	{"gnss_report?", "EVERYTHING in one block (radio, gains, g2, jammer, events) to copy into a report.", "", get_gnss_report},
	{"gnss_aj_log=", "Status line every N s on the console (default 0 = off: only JAMMER ON/OFF events; held back while you type).", "gnss_aj_log=30", set_gnss_aj_log},
	{"gnss_aj_events=", "1 = print JAMMER ON/NULLED/OFF events (default), 0 = console silent (gnss_report? still lists them).", "gnss_aj_events=0", set_gnss_aj_events},
	{"gnss_aj_avg=", "Averaging factor of the STAP solver by hand: 0.05 .. 0.99 (0.5 = 2 solves of memory, 0.9 = 10); 0 = automatic (adapted from the solves).", "gnss_aj_avg=0.5", set_gnss_aj_avg},
	{"gnss_aj_avg?", "Shows whether the averaging factor is manual or automatic, and its value.", "", get_gnss_aj_avg},
	{"gnss_aj_csv=", "CSV line (AJCSV,...) every N ms, 100 to 60000; 0 = off (default).", "gnss_aj_csv=0", set_gnss_aj_csv},
	{"gnss_aj_blank=", "Pulse blanker (bitstream 4.6): 0 = off (default since 4.9.3), 1 = automatic, 2 = always on.", "gnss_aj_blank=0", set_gnss_aj_blank},
	{"gnss_aj_blank_db=", "Pulse blanker threshold above the output noise [dB], 3..30, default 7.", "gnss_aj_blank_db=7", set_gnss_aj_blank_db},
	{"gnss_aj_sweep=", "1 = detect a moving (chirping) jammer and average over about 50 solves (default), 0 = off.", "gnss_aj_sweep=1", set_gnss_aj_sweep},
	{"gnss_notch?", "CW notch (crpa5): automatic / off / fixed, on or off, its frequency and the tone detector.", "", get_gnss_notch},
	{"gnss_notch=", "CW notch: 1 = automatic (default: on while a CW tone is seen), 0 = off.", "gnss_notch=1", set_gnss_notch},
	{"gnss_notch_rf=", "CW notch fixed at this RF frequency [MHz] (gnss_notch=1 back to automatic).", "gnss_notch_rf=1575.42", set_gnss_notch_rf},
	{"gnss_aj_mode=", "0 = space-time 63 + 2 image taps (default), 1 = RX2 taps, 2 = one weight (v3), 3 = no image taps (v4.0).", "gnss_aj_mode=0", set_gnss_aj_mode},
	{"gnss_log=", "Snapshot of status, band, rx gain, report, ... every N seconds (default 60; 0 = off; 300 = 5 min), saved with a time stamp (SD build) or printed between GNSS_LOG BEGIN/END lines.", "gnss_log=60", set_gnss_log},
	{"gnss_log?", "Snapshot logger state: period, target, count, last file, clock.", "", get_gnss_log},
	{"gnss_log_now=", "1 = write one snapshot right now.", "gnss_log_now=1", set_gnss_log_now},
	{"gnss_time=", "Set the clock used for the log time stamps (no real-time clock on the board): gnss_time=2026 11 5 14 30 0", "gnss_time=2026 11 5 14 30 0", set_gnss_time},
	{"gnss_band?", "RF profile and which GNSS signals it carries (GPS, Galileo, BeiDou, GLONASS).", "", get_gnss_band},
	{"gnss_band=", "0..6 = fixed preset (LO, rate, bandwidth, TX attenuation, RX gain), 0 = default 1569.42 MHz / 61.44 / 56 / 30 / 50; 7 = custom, gnss_band=7,LO,rate,BW,att,gain sets and applies your own values (saved); gnss_band? lists them. TX is switched on (gnss_tx=1) every time.", "gnss_band=7,1575.42,61.44,56,30,50", set_gnss_band},
	{"gnss_band7?", "Custom band 7: the stored values, where they are saved, and whether band 7 is in use.", "", get_gnss_band7},
	{"gnss_band7=", "Custom band 7 values <LO MHz>,<MS/s>,<BW MHz>,<TX1 att dB>,<RX gain dB> (leave out the tail to keep it); saved; applied at once if band 7 is in use.", "gnss_band7=1575.42,61.44,56,30,50", set_gnss_band7},
	{"gnss_band_boot?", "Band used at start-up.", "", get_gnss_band_boot},
	{"gnss_band_boot=", "Band used at start-up, 0..7 (saved). 0 = SMI-PI default.", "gnss_band_boot=7", set_gnss_band_boot},
	{"vcxo?", "40 MHz reference VCXO (design A, axi_vcxo_ctrl): mode, DAC value, lock.", "", get_vcxo},
	{"vcxo_dac=", "Fixed VCXO DAC value 0..65535 (default 23000, as design A under Linux).", "vcxo_dac=23000", set_vcxo_dac},
	{"vcxo_ref=", "0 = fixed DAC value, 1 = lock the VCXO to the 10 MHz input, 2 = lock to the PPS input.", "vcxo_ref=0", set_vcxo_ref},
	{"gnss_rx_gain?", "Gets RX1/RX2 gain mode and gain [dB].", "", get_gnss_rx_gain},
	{"gnss_rx_gain=", "Sets RX1 AND RX2 to the same gain [dB] (also the auto reference).", "gnss_rx_gain=40", set_gnss_rx_gain},
	{"gnss_rx_agc=", "0 = manual locked, 1 = AD9361 AGC per channel, 2 = matched auto (SMI-PI).", "gnss_rx_agc=2", set_gnss_rx_agc},
	/* GNSS-CRPA MOD-7: the DDR round trip. Exercises axi_ad9361_dac_dma,
	 * util_upack2 and the util_rfifo DATA path, none of which had ever moved a
	 * real sample. See gnss_txdma.h. */
	{"gnss_ddr_tx?", "Gets the DDR round-trip replay state.", "", get_gnss_ddr_tx},
	{"gnss_ddr_tx=", "1 = capture RX to DDR and replay it cyclically on TX1, 0 = stop.", "gnss_ddr_tx=0", set_gnss_ddr_tx},
};
const char cmd_no = (sizeof(cmd_list) / sizeof(command));

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
extern struct dds_state dds_st;
extern struct ad9361_rf_phy *ad9361_phy;

/* GNSS-CRPA MOD-6: tracks whether the console has enabled retransmission.
 * Mirrors what was commanded; gnss_status? reports what the HARDWARE says. */
static int gnss_tx_on = 0;

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-6. Enable or disable the RX1 -> PL -> TX1 retransmit.
 *
 * SAFETY ORDERING. Attenuation is driven to the hardware maximum (89.75 dB,
 * about -83 dBm out) BEFORE any sample path is opened, and again after
 * gnss_l1_enable_tx(), because that function deliberately leaves attenuation at
 * its 30 dB "safe start" -- about -23 dBm, which is 60 dB hotter than we want
 * for a first conducted test into a GNSS module.
 *
 * Step power up afterwards with tx1_attenuation= (SMALLER number = LOUDER).
 * gnss_tx=0 silences the transmitter in one command and is the abort path.
 *
 * This is a procedural interlock, not an RF safety device. It cannot detect
 * what is physically attached to TX1.
*******************************************************************************/
void set_gnss_tx(double* param, char param_no)
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
		/* Quietest possible BEFORE the data path opens. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_L1_TX_MAX_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_tx: attenuation not confirmed; NOT transmitting\n");
			return;
		}
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
		/* SMI-PI v4.1: then straight to the working level (70 dB, measured
		 * to give a 3D fix, see GNSS_TX_START_ATTEN_MDB), still BEFORE the
		 * data path opens. At 89.75 dB the receiver sees almost nothing, and
		 * every second spent stepping down by hand added to its time to
		 * fix. tx1_attenuation= still changes it afterwards. */
		if(gnss_l1_set_tx_attenuation(ad9361_phy,
				GNSS_TX_START_ATTEN_MDB) != GNSS_L1_OK) {
			console_print("gnss_tx: attenuation not confirmed; NOT transmitting\n");
			(void)gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
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

		gnss_tx_on = 1;
		/* Anti-jam comes with the transmitter unless the operator switched
		 * it off (gnss_smi=0): one command, gnss_tx=1, is the whole start. */
		if(!gnss_aj_user_off() && !gnss_aj_enabled())
			gnss_aj_enable(1);
		console_print("GNSS_TX: ON  - %s is being retransmitted on TX1 at "
			      "%d mdB attenuation.\n",
			      (char*)(gnss_aj_enabled() ? "the ANTI-JAM output (RX1+RX2; bypassed = RX1 while no jammer)" : "RX1"),
			      (long)GNSS_TX_START_ATTEN_MDB);
		console_print("         Change it with tx1_attenuation= (smaller = "
			      "louder). Abort with gnss_tx=0\n");
		gnss_aj_note_tx(1);
	} else {
		gnss_l1_set_dac_source(ad9361_phy, AXI_DAC_DATA_SEL_ZERO);
		gnss_pt_set_mute(1);
		gnss_l1_set_tx_attenuation(ad9361_phy, GNSS_L1_TX_MAX_ATTEN_MDB);
		gnss_tx_on = 0;
		console_print("GNSS_TX: OFF - DAC source ZERO, passthrough muted, "
			      "attenuation at maximum.\n");
		gnss_aj_note_tx(0);
	}
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-6. Report the retransmit state, read back from hardware.
*******************************************************************************/
void get_gnss_tx(double* param, char param_no)
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

	console_print("GNSS_TX: %s (read from hardware) - pass_en = %d, "
		      "dac_enable i/q = %d / %d, TX1 attenuation %d mdB\n",
		      transmitting ? "TRANSMITTING" : "silent",
		      (long)pass_en, (long)dac_i, (long)dac_q, (long)atten);
	console_print("         gnss_tx= console flag: %s%s\n",
		      gnss_tx_on ? "ON" : "OFF",
		      (transmitting && !gnss_tx_on)
		          ? "   (started at boot, not by a console command)" : "");
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
}

/**************************************************************************//***
 * @brief GNSS-CRPA 4.9.14: TX2 state. TX2's source is the CRPA output (the
 * samples TX1 carries), never raw RX2; it is muted unless gnss_tx2=1.
*******************************************************************************/
void get_gnss_tx2(double* param, char param_no)
{
	uint32_t ver = gnss_pt_read(GNSS_PT_REG_VERSION);
	uint32_t c   = gnss_pt_read(GNSS_PT_REG_CONTROL);
	uint32_t att = 0;

	(void)param; (void)param_no;
	ad9361_get_tx_attenuation(ad9361_phy, 1, &att);
	if(ver < GNSS_PT_VERSION_TX2_MUTE) {
		console_print("gnss_tx2=%d  (bitstream %d.%d cannot mute TX2: it carries %s, "
			      "attenuation %d mdB)\n", 1L, (long)(ver >> 16), (long)(ver & 0xFFFFU),
			      (char*)((c & GNSS_PT_CTRL_CH1_COPY_OLD) ? "a copy of the CRPA output" : "RAW RX2"),
			      (long)att);
		return;
	}
	console_print("gnss_tx2=%d  (TX2 %s; source %s; attenuation %d mdB)\n",
		      (long)gnss_pt_tx2_on(),
		      (char*)(gnss_pt_tx2_on() ? "LIVE" : "MUTED, digital zero"),
		      (char*)((c & GNSS_PT_CTRL_CH1_RAW) ? "RAW RX2 (diagnostic)" : "CRPA output = TX1 samples"),
		      (long)att);
}

/**************************************************************************//***
 * @brief GNSS-CRPA 4.9.14: gnss_tx2=0 mutes TX2 (the default), gnss_tx2=1
 * lets out a copy of the CRPA output. The attenuation is not touched: it
 * stays at GNSS_L1_TX2_ATTEN_MDB (89.75 dB).
*******************************************************************************/
void set_gnss_tx2(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_tx2= needs 0 or 1\n");
		return;
	}
	gnss_pt_set_tx2((int)param[0] != 0);
	get_gnss_tx2(param, param_no);
}

/**************************************************************************//***
 * @brief SMI-PI v4.1 anti-jam: one-line state.
*******************************************************************************/
void get_gnss_smi(double* param, char param_no)
{
	uint32_t ver = gnss_pt_read(GNSS_PT_REG_VERSION);

	(void)param; (void)param_no;
	if(ver < 0x00040001U) {
		console_print("GNSS_SMI: bitstream v%d.%d has no SMI-PI v4.1 filter "
			      "(this firmware needs v4.1)\n",
			      (long)(ver >> 16), (long)(ver & 0xFFFFU));
		return;
	}
	gnss_aj_print_short();
}

/**************************************************************************//***
 * @brief SMI-PI v4.1. Anti-jam on (1) or off (0). On: TX1 carries the
 * filter output -- bypassed (= RX1) while there is no jammer, the space-time
 * filter with 63 + 2 weights computed by the ARM while there is one
 * (gnss_aj.c) -- and both RX gains are matched. It does not open the transmitter; that
 * is still gnss_tx=1, with its attenuation interlock (which also switches
 * anti-jam on unless gnss_smi=0 was given). RX2 must have its antenna.
*******************************************************************************/
void set_gnss_smi(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_smi= needs 0 or 1\n");
		return;
	}
	if(gnss_pt_read(GNSS_PT_REG_VERSION) < 0x00040001U) {
		console_print("gnss_smi: this bitstream has no SMI-PI v4.1 filter\n");
		return;
	}
	gnss_aj_command((int)param[0] != 0);
	get_gnss_smi(param, param_no);
}

/**************************************************************************//***
 * @brief SMI-PI v4.1. Anti-jam status, full report, log period, CSV, filter
 * mode and RF profile.
*******************************************************************************/
void get_gnss_aj(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_aj_print();
}

void get_gnss_report(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_aj_report();
}

void set_gnss_aj_avg(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 0.99 || (param[0] > 0.0 && param[0] < 0.05)) {
		console_print("gnss_aj_avg= needs 0 (automatic), or a factor 0.05 to 0.99. Example: gnss_aj_avg=0.5\n");
		return;
	}
	gnss_aj_set_avg(param[0], param[0] == 0.0);
	gnss_settings_save(-1, (int)(param[0] * 1000.0 + 0.5));   /* crpa5 */
}

void get_gnss_aj_avg(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_aj_avg_print();
}

void set_gnss_aj_csv(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 60000.0) {
		console_print("gnss_aj_csv= needs a period in ms, 100 to 60000, or 0 = off, e.g. gnss_aj_csv=500\n");
		return;
	}
	gnss_aj_set_csv((int)param[0]);
	if((int)param[0] == 0)
		console_print("GNSS_AJ: CSV off\n");
	else
		console_print("GNSS_AJ: CSV line every %d ms (at least 100). Log the PuTTY session to a file\n"
			      "         and keep the lines starting with AJCSV, (columns in the first one).\n",
			      (long)((int)param[0] < 100 ? 100 : (int)param[0]));
}

void set_gnss_aj_events(double* param, char param_no)
{
	if(param_no < 1 || (param[0] != 0.0 && param[0] != 1.0)) {
		console_print("gnss_aj_events= needs 0 (silent) or 1 (events printed), e.g. gnss_aj_events=0\n");
		return;
	}
	gnss_aj_set_events((int)param[0]);
	console_print("GNSS_AJ: jammer events %s\n",
		      (char*)((int)param[0] ? "printed" : "NOT printed (gnss_report? still lists them; gnss_aj_events=1 to print)"));
}

void set_gnss_aj_log(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 3600.0) {
		console_print("gnss_aj_log= needs seconds, 0 to 3600 (0 = only JAMMER ON/OFF), e.g. gnss_aj_log=30\n");
		return;
	}
	gnss_aj_set_log((int)param[0]);
	console_print("GNSS_AJ: status line every %d s%s\n", (long)param[0],
		      (char*)((int)param[0] == 0 ? " (off; JAMMER ON/OFF events are still printed)" : ""));
}

void set_gnss_aj_mode(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 3.0) {
		console_print("gnss_aj_mode= needs 0 (space-time 63 + 2 image taps), 1 (RX2 taps), 2 (one weight, v3)\n"
			      "              or 3 (space-time without image taps, v4.0)\n");
		return;
	}
	gnss_aj_set_mode((int)param[0]);
	gnss_aj_print_short();
}

void set_gnss_aj_blank(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 2.0) {
		console_print("gnss_aj_blank= needs 1 (automatic, default), 0 (off) or 2 (always on)\n");
		return;
	}
	if(!gnss_aj_blank_available())
		console_print("the bitstream has no pulse blanker (needs 4.6); the setting is kept for when it has\n");
	gnss_aj_set_blank((int)param[0]);
	gnss_aj_print_short();
}

void set_gnss_aj_blank_db(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 3.0 || param[0] > 30.0) {
		console_print("gnss_aj_blank_db= needs 3..30 (dB above the output noise, default 7)\n");
		return;
	}
	gnss_aj_set_blank_db(param[0]);
	gnss_aj_print_short();
}

void set_gnss_aj_sweep(double* param, char param_no)
{
	if(param_no < 1 || (param[0] != 0.0 && param[0] != 1.0)) {
		console_print("gnss_aj_sweep= needs 1 (long memory for a moving jammer, default) or 0 (off)\n");
		return;
	}
	gnss_aj_set_sweep((int)param[0]);
	gnss_aj_print_short();
}

void get_gnss_band(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_band_print();
}

void set_gnss_band(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > (double)(GNSS_BAND_COUNT - 1)) {
		console_print("gnss_band= needs 0, 1, 2, 3, 4, 5, 6 or 7 (gnss_band? lists them)\n"
			      "  7 = custom: gnss_band=7,<LO MHz>,<MS/s>,<BW MHz>,<TX1 att dB>,<RX gain dB>\n");
		return;
	}
	if((int)param[0] == GNSS_BAND_CUSTOM && param_no > 1) {
		/* antsdr-fw-patch: gnss_band=7,LO,rate,BW,att,gain -- new custom values */
		if(gnss_band7_values(&param[1], param_no - 1) != 0)
			return;                     /* refused: nothing changed */
	} else if(param_no > 1)
		console_print("gnss_band: the presets 0..6 are fixed (4.9.5): any value after the number is ignored.\n"
			      "  Band 7 is the custom one: gnss_band=7,<LO MHz>,<MS/s>,<BW MHz>,<TX1 att dB>,<RX gain dB>\n");
	if(gnss_band_set((int)param[0]) != 0)
		console_print("gnss_band: the AD9361 refused part of the setting; check gnss_status?\n");
	gnss_band_print();
	gnss_tx_always();
}

void set_gnss_log(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 86400.0) {
		console_print("gnss_log= needs a period in seconds, 0 (off) to 86400, e.g. gnss_log=60 or gnss_log=300\n");
		return;
	}
	gnss_log_set_period((unsigned)param[0]);
	gnss_log_print_state();
}

void get_gnss_log(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_log_print_state();
}

void set_gnss_log_now(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_log_now();
}

void set_gnss_time(double* param, char param_no)
{
	if(param_no < 6 || param[0] < 2000.0 || param[0] > 2099.0 || param[1] < 1.0 || param[1] > 12.0 ||
	   param[2] < 1.0 || param[2] > 31.0 || param[3] < 0.0 || param[3] > 23.0 || param[4] < 0.0 || param[4] > 59.0 ||
	   param[5] < 0.0 || param[5] > 59.0) {
		console_print("gnss_time= needs year month day hour minute second, e.g. gnss_time=2026 11 5 14 30 0\n");
		return;
	}
	gnss_log_set_clock((int)param[0], (int)param[1], (int)param[2], (int)param[3], (int)param[4], (int)param[5]);
	console_print("GNSS_LOG: clock set\n");
	gnss_log_print_state();
}

/* 4.9.5: gnss_tx=1 always. Called at start-up and after every gnss_band=: switches TX on if it is not on, then
 * applies the band's TX attenuation and RX gain. gnss_tx=0 still silences TX until the next boot or gnss_band=.
 * CONDUCTED, ATTENUATED COAX ONLY: the firmware cannot see what is attached to TX1. */
void gnss_tx_always(void)
{
	double one = 1.0;
	if(!gnss_tx_on)
		set_gnss_tx(&one, 1);
	if(gnss_tx_on)
		gnss_band_levels();
}

/**************************************************************************//***
 * @brief RX gain control for the SMI-PI pair (gnss_smi_agc.c).
*******************************************************************************/
void get_gnss_rx_gain(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_rxg_print();
}

void set_gnss_rx_gain(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 1.0 || param[0] > 76.0) {
		console_print("gnss_rx_gain= needs a gain in dB, 1 to 76, e.g. gnss_rx_gain=40\n");
		return;
	}
	gnss_rxg_set_gain((int32_t)param[0]);
	gnss_levels_adopt();
	gnss_rxg_print();
}

void set_gnss_rx_agc(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 2.0) {
		console_print("gnss_rx_agc= needs 0 (manual), 1 (AD9361 AGC) or 2 (matched auto)\n");
		return;
	}
	gnss_rxg_set_mode((int)param[0]);
	gnss_rxg_print();
	gnss_settings_save((int)param[0], -1);   /* crpa5 */
}

/**************************************************************************//***
 * @brief SMI-PI v4.1. Hold (1) the filter as it is, or automatic (0).
*******************************************************************************/
/* crpa5: CW notch */
void get_gnss_notch(double* param, char param_no)
{
	(void)param; (void)param_no;
	gnss_notch_print();
}

void set_gnss_notch(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 0.0 || param[0] > 1.0) {
		console_print("gnss_notch= needs 1 (automatic) or 0 (off); a fixed notch: gnss_notch_rf=<RF MHz>\n");
		return;
	}
	gnss_notch_set_mode((int)param[0]);
	gnss_notch_print();
}

void set_gnss_notch_rf(double* param, char param_no)
{
	if(param_no < 1 || param[0] < 70.0 || param[0] > 6000.0) {
		console_print("gnss_notch_rf= needs an RF frequency in MHz, e.g. gnss_notch_rf=1575.42\n");
		return;
	}
	if(gnss_notch_set_rf(param[0]) != 0) return;
	gnss_notch_print();
}

void set_gnss_smi_freeze(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_smi_freeze= needs 0 or 1\n");
		return;
	}
	gnss_aj_freeze((int)param[0] != 0);
	get_gnss_smi(param, param_no);
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
	gnss_info_menu();
}
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
		/* The anti-jam engine captures on the same RX DMA: let its transfer
		 * in flight finish first (it pauses while the replay runs). */
		gnss_aj_dma_release();
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
void get_register(double* param, char param_no) // "register?" command
{
	uint16_t reg_addr;
	uint8_t reg_val;
	struct spi_device spi;

	if(param_no >= 1)
	{
		spi.id_no = 0;
		reg_addr = param[0];
		reg_val = ad9361_spi_read(&spi, reg_addr);
		console_print("register[0x%x]=0x%x\n", reg_addr, reg_val);
	}
	else
		show_invalid_param_message(1);
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

	console_print("tx_lo_freq=%s\n", gnss_fmt((double)lo_freq_hz / 1e6, 4));
}

/* LO helpers shared with radio= (gnss_aj.c): the E310's RF switch follows the LO. */
int gnss_cmd_tx_lo_hz(uint64_t lo_freq_hz)
{
#ifdef ANTSDR_E310
	/* set tx rf switch */
	if(lo_freq_hz <= 3000000000ULL){
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
	return ad9361_set_tx_lo_freq(ad9361_phy, lo_freq_hz) ? 1 : 0;
}

int gnss_cmd_rx_lo_hz(uint64_t lo_freq_hz)
{
#ifdef ANTSDR_E310
	/* set rx rf swicth */
	if(lo_freq_hz <= 3000000000ULL){
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
	return ad9361_set_rx_lo_freq(ad9361_phy, lo_freq_hz) ? 1 : 0;
}

static void range_msg(const char *what, const char *range)
{
	console_print("%s is outside the AD9361 range (%s); nothing was changed.\n", (char*)what, (char*)range);
}

/**************************************************************************//***
 * @brief Sets the TX LO frequency. 1575.42 = MHz (any decimals), or Hz if >= 100000.
*******************************************************************************/
void set_tx_lo_freq(double* param, char param_no) // "tx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_lo_hz(param[0], &lo_freq_hz)) {
		range_msg("tx_lo_freq", "70 to 6000 MHz");
		return;
	}
	gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
	gnss_cmd_tx_lo_hz(lo_freq_hz);
	get_tx_lo_freq(param, 0);
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
 * @brief Sets the sampling frequency: 30.72 = MS/s (any decimals), or Hz if >= 1000.
*******************************************************************************/
void set_tx_samp_freq(double* param, char param_no) // "tx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_rate_hz(param[0], &sampling_freq_hz)) {
		range_msg("tx_samp_freq", "2.083333 to 61.44 MS/s");
		return;
	}
	gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
	ad9361_set_tx_sampling_freq(ad9361_phy, sampling_freq_hz);
	ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("tx_samp_freq=%d\n", sampling_freq_hz);
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

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_bw_hz(param[0], &bandwidth_hz)) {
		range_msg("tx_rf_bandwidth", "0.2 to 56 MHz");
		return;
	}
	gnss_aj_radio_manual();
	ad9361_set_tx_rf_bandwidth(ad9361_phy, bandwidth_hz);
	ad9361_get_tx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("tx_rf_bandwidth=%d\n", bandwidth_hz);
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
void get_radio(double* param, char param_no) // "radio?" command
{
	gnss_radio_print();
}

void set_radio(double* param, char param_no) // "radio=" command
{
	gnss_radio_cmd(param, param_no);
}

/* "tx_att_db=70.5": dB with decimals, instead of tx1_attenuation= in milli-dB */
void set_tx_att_db(double* param, char param_no)
{
	uint32_t mdb;

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_att_mdb(param[0], &mdb)) {
		range_msg("tx_att_db", "0 to 89.75 dB");
		return;
	}
	ad9361_set_tx_attenuation(ad9361_phy, 0, mdb);
	ad9361_get_tx_attenuation(ad9361_phy, 0, &mdb);
	console_print("tx1_attenuation=%d mdB (%s dB)\n", mdb, gnss_fmt((double)mdb / 1000.0, 2));
}

void set_tx1_attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb;

	if(param_no >= 1)
	{
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 0, attenuation_mdb);
		gnss_levels_adopt();
		console_print("tx1_attenuation=%d\n", attenuation_mdb);
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
		/* 4.9.13: TX2 is not used and is fixed at the quietest setting (GNSS_L1_TX2_ATTEN_MDB). */
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 1, GNSS_L1_TX2_ATTEN_MDB);
		if(attenuation_mdb != GNSS_L1_TX2_ATTEN_MDB)
			console_print("tx2_attenuation: TX2 is not used in this build and stays at %d mdB (asked %d)\n",
				      (int)GNSS_L1_TX2_ATTEN_MDB, (int)attenuation_mdb);
		console_print("tx2_attenuation=%d\n", (int)GNSS_L1_TX2_ATTEN_MDB);
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
	if(param_no >= 1) gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
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
	console_print("rx_lo_freq=%s\n", gnss_fmt((double)lo_freq_hz / 1e6, 4));
}

/**************************************************************************//***
 * @brief Sets the RX LO frequency. 1575.42 = MHz (any decimals), or Hz if >= 100000.
*******************************************************************************/
void set_rx_lo_freq(double* param, char param_no) // "rx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_lo_hz(param[0], &lo_freq_hz)) {
		range_msg("rx_lo_freq", "70 to 6000 MHz");
		return;
	}
	gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
	gnss_cmd_rx_lo_hz(lo_freq_hz);
	get_rx_lo_freq(param, 0);
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
 * @brief Sets the RX sampling frequency: 30.72 = MS/s (any decimals), or Hz if >= 1000.
*******************************************************************************/
void set_rx_samp_freq(double* param, char param_no) // "rx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_rate_hz(param[0], &sampling_freq_hz)) {
		range_msg("rx_samp_freq", "2.083333 to 61.44 MS/s");
		return;
	}
	gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
	ad9361_set_rx_sampling_freq(ad9361_phy, sampling_freq_hz);
	ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("rx_samp_freq=%d\n", sampling_freq_hz);
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

	if(param_no < 1) { show_invalid_param_message(1); return; }
	if(gnss_radio_bw_hz(param[0], &bandwidth_hz)) {
		range_msg("rx_rf_bandwidth", "0.2 to 56 MHz");
		return;
	}
	gnss_aj_radio_manual();
	ad9361_set_rx_rf_bandwidth(ad9361_phy, bandwidth_hz);
	ad9361_get_rx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("rx_rf_bandwidth=%d\n", bandwidth_hz);
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
	if(param_no >= 1) gnss_aj_radio_manual();   /* keep it: the radio guard must not put the profile back */
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
