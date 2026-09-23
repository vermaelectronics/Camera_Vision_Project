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
/* GNSS-CRPA MOD-7 */
#include "gnss_txdma.h"
#include "axi_dmac.h"
/* GNSS-CRPA MOD-10 */
#include "gnss_info.h"

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
	{"tx2_attenuation?", "Gets current TX2 attenuation [mdB].", "", get_tx2_attenuation},
	{"tx2_attenuation=", "Sets the TX2 attenuation [mdB].", "", set_tx2_attenuation},
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
	{"gnss_tx?", "Gets the GNSS L1 retransmit state.", "", get_gnss_tx},
	{"gnss_tx=", "1 = retransmit RX1 on TX1, 0 = silence TX1.", "gnss_tx=0", set_gnss_tx},
	{"gnss_status?", "Prints AD9361 and gnss_passthrough runtime state.", "", get_gnss_status},
	/* GNSS-CRPA MOD-7: the DDR round trip. Exercises axi_ad9361_dac_dma,
	 * util_upack2 and the util_rfifo DATA path, none of which had ever moved a
	 * real sample. See gnss_txdma.h. */
	{"gnss_ddr_tx?", "Gets the DDR round-trip replay state.", "", get_gnss_ddr_tx},
	{"gnss_ddr_tx=", "1 = capture RX to DDR and replay it cyclically on TX1, 0 = stop.", "gnss_ddr_tx=0", set_gnss_ddr_tx},
	/* GNSS-CRPA MOD-11: the power-inversion nulling core's adaptation step
	 * size. Only live on a v1.2+ gnss_passthrough bitstream. */
	{"gnss_crpa_alpha?", "Gets the CRPA adaptation step size (alpha).", "", get_gnss_crpa_alpha},
	{"gnss_crpa_alpha=", "Sets the CRPA adaptation step size (alpha).", "gnss_crpa_alpha=1.0", set_gnss_crpa_alpha},
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
		console_print("GNSS_TX: ON  - RX1 is being retransmitted on TX1 at "
			      "%d mdB attenuation (quietest).\n",
			      (long)GNSS_L1_TX_MAX_ATTEN_MDB);
		console_print("         Raise power with tx1_attenuation= (smaller = "
			      "louder). Abort with gnss_tx=0\n");
	} else {
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
 * @brief GNSS-CRPA MOD-11. Set the CRPA nulling core's adaptation step size
 *        (alpha). Only affects a v1.2+ gnss_passthrough bitstream; see
 *        gnss_pt_print_state() to confirm which core is actually loaded.
*******************************************************************************/
void set_gnss_crpa_alpha(double* param, char param_no)
{
	if(param_no < 1) {
		console_print("gnss_crpa_alpha= needs a value, e.g. gnss_crpa_alpha=1.0\n");
		return;
	}
	if(param[0] < 0.0) {
		console_print("gnss_crpa_alpha=: negative alpha is not representable, "
			      "clamping to 0\n");
	}

	gnss_pt_set_crpa_alpha(param[0]);
	console_print("GNSS_CRPA_ALPHA: set to %.4f (raw=%u)\n",
		      gnss_pt_get_crpa_alpha(),
		      (unsigned)gnss_pt_get_crpa_alpha_raw());
}

/**************************************************************************//***
 * @brief GNSS-CRPA MOD-11. Report the CRPA adaptation step size (alpha),
 *        read back from hardware.
*******************************************************************************/
void get_gnss_crpa_alpha(double* param, char param_no)
{
	uint32_t version = gnss_pt_read(GNSS_PT_REG_VERSION);

	console_print("GNSS_CRPA_ALPHA: %.4f (raw=%u)\n",
		      gnss_pt_get_crpa_alpha(),
		      (unsigned)gnss_pt_get_crpa_alpha_raw());
	if(version != GNSS_PT_EXPECTED_VERSION) {
		console_print("                 WARNING: bitstream reports version "
			      "%lu.%lu, not %lu.%lu -- the CRPA core may not be "
			      "present, so this value may not be consumed by "
			      "anything.\n",
			      (unsigned long)(version >> 16), (unsigned long)(version & 0xFFFFU),
			      (unsigned long)(GNSS_PT_EXPECTED_VERSION >> 16),
			      (unsigned long)(GNSS_PT_EXPECTED_VERSION & 0xFFFFU));
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

	lo_freq_hz /= 1000000;
	console_print("tx_lo_freq=%d\n", (uint32_t)lo_freq_hz);
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
		lo_freq_hz = param[0];
		lo_freq_hz *= 1000000;
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
		lo_freq_hz /= 1000000;
		console_print("tx_lo_freq=%d\n", (uint32_t)lo_freq_hz);
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
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 1, attenuation_mdb);
		console_print("tx2_attenuation=%d\n", attenuation_mdb);
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
	lo_freq_hz /= 1000000;
	console_print("rx_lo_freq=%d\n", (uint32_t)lo_freq_hz);
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
		lo_freq_hz = param[0];
		lo_freq_hz *= 1000000;
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
		lo_freq_hz /= 1000000;
		console_print("rx_lo_freq=%d\n", (uint32_t)lo_freq_hz);
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
