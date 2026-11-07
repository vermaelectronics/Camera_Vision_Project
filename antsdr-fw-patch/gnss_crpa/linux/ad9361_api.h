/* AD9361 no-OS API, implemented on Linux by lx_ad9361.c on top of the
 * kernel's ad9361-phy IIO device (sysfs). Only what the GNSS firmware uses. */
#ifndef AD9361_API_H_LX
#define AD9361_API_H_LX
#include <stdint.h>
#include "no_os_gpio.h"
#include "axi_dac_core.h"
#include "axi_adc_core.h"

#define ENABLE		1
#define DISABLE		0
#define RX1		0
#define RX2		1
#define TX1		0
#define TX2		1
#define A_BALANCED	0
#define B_BALANCED	1
#define C_BALANCED	2
#define A_N		3
#define A_P		4
#define B_N		5
#define B_P		6
#define C_N		7
#define C_P		8
#define TX_MON1		9
#define TX_MON2		10
#define TX_MON1_2	11
#define TXA		0
#define TXB		1

enum rf_gain_ctrl_mode {
	RF_GAIN_MGC,
	RF_GAIN_FASTATTACK_AGC,
	RF_GAIN_SLOWATTACK_AGC,
	RF_GAIN_HYBRID_AGC
};

#define RFDC_CAL		(1 << 1)
#define TX_QUAD_CAL		(1 << 4)

#define ENSM_STATE_SLEEP_WAIT	0x0
#define ENSM_STATE_ALERT	0x5
#define ENSM_STATE_TX		0x6
#define ENSM_STATE_TX_FLUSH	0x7
#define ENSM_STATE_RX		0x8
#define ENSM_STATE_RX_FLUSH	0x9
#define ENSM_STATE_FDD		0xA
#define ENSM_STATE_FDD_FLUSH	0xB
#define ENSM_STATE_INVALID	0xFF
#define ENSM_STATE_SLEEP	0x80

struct rf_rx_gain {
	uint32_t ant;
	int32_t gain_db;
	uint32_t fgt_lmt_index;
	uint32_t lmt_gain;
	uint32_t lpf_gain;
	uint32_t digital_gain;
	uint32_t lna_index;
	uint32_t tia_index;
	uint32_t mixer_index;
};

struct rf_rssi {
	uint32_t ant;
	uint32_t symbol;
	uint32_t preamble;
	int32_t multiplier;
	uint8_t duration;
};

struct ad9361_rf_phy {
	struct axi_dac *tx_dac;
	struct axi_adc *rx_adc;
	struct no_os_gpio_desc *gpio_desc_rx1_ctrl_h, *gpio_desc_rx1_ctrl_l;
	struct no_os_gpio_desc *gpio_desc_tx1_ctrl_h, *gpio_desc_tx1_ctrl_l;
	struct no_os_gpio_desc *gpio_desc_rx2_ctrl_h, *gpio_desc_rx2_ctrl_l;
	struct no_os_gpio_desc *gpio_desc_tx2_ctrl_h, *gpio_desc_tx2_ctrl_l;
	uint8_t bbdc_track_en, rfdc_track_en, quad_track_en;
};

struct spi_device { uint8_t id_no; };

int32_t ad9361_set_rx_rf_gain(struct ad9361_rf_phy *phy, uint8_t ch, int32_t gain_db);
int32_t ad9361_get_rx_rf_gain(struct ad9361_rf_phy *phy, uint8_t ch, int32_t *gain_db);
int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *phy, uint32_t bandwidth_hz);
int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *phy, uint32_t *bandwidth_hz);
int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz);
int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *phy, uint32_t *sampling_freq_hz);
int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *phy, uint64_t lo_freq_hz);
int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *phy, uint64_t *lo_freq_hz);
int32_t ad9361_get_rx_rssi(struct ad9361_rf_phy *phy, uint8_t ch, struct rf_rssi *rssi);
int32_t ad9361_set_rx_gain_control_mode(struct ad9361_rf_phy *phy, uint8_t ch, uint8_t gc_mode);
int32_t ad9361_get_rx_gain_control_mode(struct ad9361_rf_phy *phy, uint8_t ch, uint8_t *gc_mode);
int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t en_dis);
int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t *en_dis);
int32_t ad9361_set_rx_rf_port_input(struct ad9361_rf_phy *phy, uint32_t mode);
int32_t ad9361_get_rx_rf_port_input(struct ad9361_rf_phy *phy, uint32_t *mode);
int32_t ad9361_set_tx_attenuation(struct ad9361_rf_phy *phy, uint8_t ch, uint32_t attenuation_mdb);
int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *phy, uint8_t ch, uint32_t *attenuation_mdb);
int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *phy, uint32_t bandwidth_hz);
int32_t ad9361_get_tx_rf_bandwidth(struct ad9361_rf_phy *phy, uint32_t *bandwidth_hz);
int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz);
int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *phy, uint32_t *sampling_freq_hz);
int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *phy, uint64_t lo_freq_hz);
int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *phy, uint64_t *lo_freq_hz);
int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t en_dis);
int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *phy, uint8_t *en_dis);
int32_t ad9361_set_tx_rf_port_output(struct ad9361_rf_phy *phy, uint32_t mode);
int32_t ad9361_get_tx_rf_port_output(struct ad9361_rf_phy *phy, uint32_t *mode);
int32_t ad9361_do_calib_run(struct ad9361_rf_phy *phy, uint32_t cal, int32_t arg);
uint8_t ad9361_ensm_get_state(struct ad9361_rf_phy *phy);
int32_t ad9361_spi_read(void *spi, uint32_t reg);

int     lx_ad9361_open(struct ad9361_rf_phy **phy);
#endif
