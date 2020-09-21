/*********************
 *      INCLUDES
 *********************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

#include "as3933_hal.h"
#include "as3933_defs.h"
#include "as3933_cfg.h"
#include "as3933.h"
#include "foreach.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static uint16_t convert_hex_to_manchester(uint8_t hex);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "as3933";
as3933_t as3933;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void as3933_init_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	foreach (as3933_reg_setting_t *cfg, as3933_config_register) //as3933_default_register) //
	{
		as3933_write_byte(cfg->addr, cfg->data);
	}
	ESP_LOGD(TAG, "Set init settings.");
}

void as3933_read_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	as3933_read_bytes(0, (uint8_t*) &as3933, sizeof(as3933));
}

void as3933_print_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r0   ", as3933.r0.reg, BYTE_TO_BINARY(as3933.r0.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r1   ", as3933.r1.reg, BYTE_TO_BINARY(as3933.r1.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r2   ", as3933.r2.reg, BYTE_TO_BINARY(as3933.r2.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r3   ", as3933.r3.reg, BYTE_TO_BINARY(as3933.r3.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r4   ", as3933.r4.reg, BYTE_TO_BINARY(as3933.r4.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r5   ", as3933.r5.patt2b, BYTE_TO_BINARY(as3933.r5.patt2b));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r6   ", as3933.r6.patt1b, BYTE_TO_BINARY(as3933.r6.patt1b));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r7   ", as3933.r7.reg, BYTE_TO_BINARY(as3933.r7.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r8   ", as3933.r8.reg, BYTE_TO_BINARY(as3933.r8.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r9   ", as3933.r9.reg, BYTE_TO_BINARY(as3933.r9.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r10  ", as3933.r10.reg, BYTE_TO_BINARY(as3933.r10.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r11  ", as3933.r11.reg, BYTE_TO_BINARY(as3933.r11.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r12  ", as3933.r12.reg, BYTE_TO_BINARY(as3933.r12.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r13  ", as3933.r13.f_wake, BYTE_TO_BINARY(as3933.r13.f_wake));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r14  ", as3933.r14.reg, BYTE_TO_BINARY(as3933.r14.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r15  ", as3933.r15.reg, BYTE_TO_BINARY(as3933.r15.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r16  ", as3933.r16.reg, BYTE_TO_BINARY(as3933.r16.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r17  ", as3933.r17.reg, BYTE_TO_BINARY(as3933.r17.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r18  ", as3933.r18.reg, BYTE_TO_BINARY(as3933.r18.reg));
	ESP_LOGI(TAG, "%s %#04x   " BYTE_TO_BINARY_PATTERN, "r19  ", as3933.r19.reg, BYTE_TO_BINARY(as3933.r19.reg));
}

void as3933_reset()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_cmd(PRESET_DEFAULT);
}

void as3933_crystal_osc_select(bool select)
{
}

void as3933_set_channel(uint8_t channel, bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r0_t r0;
	as3933_read_byte(R0, &r0.reg);

	switch (channel)
	{
		case 0:
			r0.en_a1 = value;
			r0.en_a2 = value;
			r0.en_a3 = value;
			break;
		case 1:
			r0.en_a1 = value;
			break;
		case 2:
			r0.en_a2 = value;
			break;
		case 3:
			r0.en_a3 = value;
			break;
		default:
			break;
	}
	as3933_write_byte(R0, r0.reg);
}

bool as3933_get_channel(uint8_t channel)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r0_t r0;
	as3933_read_byte(R0, &r0.reg);

	switch (channel)
	{
		case 1:
			return r0.en_a1;
			break;
		case 2:
			return r0.en_a2;
			break;
		case 3:
			return r0.en_a3;
			break;
		default:
			return NULL;
			break;
	}
}

void as3933_set_mask_data_before_wu(bool select)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r0_t r0;
	as3933_read_byte(R1, &r0.reg);

	r0.dat_mask = select;
	as3933_write_byte(R1, r0.reg);
}

void as3933_set_manchaster_decode(bool select)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);

	r1.en_manch = select;
	as3933_write_byte(R1, r1.reg);
}

void as3933_set_patern_correlation(wake_up_mode_t mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);

	switch (mode)
	{
		case WK_FREQ_DET_ONLY:
			r1.en_wpat = false;
			break;
		case WK_SINGLE_PATTERN:
			r1.en_wpat = true;
			break;
	}

	as3933_write_byte(R1, r1.reg);
}

void as3933_set_wakeup_pattern_16bit(uint16_t wakeup_node_id)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_write_byte(R6, (uint8_t) wakeup_node_id);
	as3933_write_byte(R5, (uint8_t) (wakeup_node_id >> 8));
}

uint16_t as3933_get_wakeup_pattern_16bit()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	patt_t patt =
		{
			.r5.patt2b = 0,
			.r6.patt1b = 0 };
	ESP_ERROR_CHECK(as3933_read_byte(R5, &patt.r5.patt2b));
	ESP_ERROR_CHECK(as3933_read_byte(R6, &patt.r6.patt1b));
	return patt.reg;
}

void as3933_set_wakeup_pattern_8bit(uint8_t wakeup_node_id)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint16_t address = convert_hex_to_manchester(wakeup_node_id);
	as3933_set_wakeup_pattern_16bit(address);
}

void as3933_set_bitrate(uint8_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (value < 4)
		value = 4;
	else if (value > 32)
		value = 32;

	r7_t r7;
	as3933_read_byte(R7, &r7.reg);
	r7.t_hbit = value;
	as3933_write_byte(R7, r7.reg);
}

void as3933_set_auto_time_out(t_out_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (value > 7)
		value = 7;

	r7_t r7;
	as3933_read_byte(R7, &r7.reg);
	r7.t_out = value;
	as3933_write_byte(R7, r7.reg);
}

void as3933_band_select(band_sel_t freq)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r8_t r8;
	as3933_read_byte(R8, &r8.reg);
	r8.band_sel = freq;
	as3933_write_byte(R8, r8.reg);
}

void as3933_route_res_freq_on_dat(uint8_t channel, bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r16_t r16;
	as3933_read_byte(R16, &r16.reg);

	switch (channel)
	{
		case 0:
			r16.lc_osc_mux1 = value;
			r16.lc_osc_mux2 = value;
			r16.lc_osc_mux3 = value;
			break;
		case 1:
			r16.lc_osc_mux1 = value;
			r16.lc_osc_mux2 = false;
			r16.lc_osc_mux3 = false;
			break;
		case 2:
			r16.lc_osc_mux1 = false;
			r16.lc_osc_mux2 = value;
			r16.lc_osc_mux3 = false;
			break;
		case 3:
			r16.lc_osc_mux1 = false;
			r16.lc_osc_mux2 = false;
			r16.lc_osc_mux3 = value;
			break;
		default:
			break;
	}
	as3933_write_byte(R16, r16.reg);
}

void as3933_route_clock_on_dat(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r2_t r2;
	as3933_read_byte(R2, &r2.reg);
	r2.display_clk = value ? 0b11 : 0;
	as3933_write_byte(R2, r2.reg);

	r16_t r16;
	as3933_read_byte(R16, &r16.reg);
	r16.clock_gen_dis = value;
	as3933_write_byte(R16, r16.reg);
}

void as3933_set_xtal_osc(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);
	r1.en_xtal = value;
	as3933_write_byte(R1, r1.reg);
}

void as3933_set_agc_up_and_down(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);
	r1.agc_ud = value;
	as3933_write_byte(R1, r1.reg);
}

void as3933_set_agc_first_carrier(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);
	r1.agc_tlim = value;
	as3933_write_byte(R1, r1.reg);
}

// 0 - 31 pF
void as3933_set_capacity(uint8_t channel, uint8_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	if (value > 31)
		value = 31;

	switch (channel)
	{
		case 1:
			{
				r17_t r17 =
					{
						.cap_ch1 = value };
				as3933_write_byte(R17, r17.reg);
				break;
			}
		case 2:
			{
				r18_t r18 =
					{
						.cap_ch2 = value };
				as3933_write_byte(R18, r18.reg);
				break;
			}
		case 3:
			{
				r19_t r19 =
					{
						.cap_ch3 = value };
				as3933_write_byte(R19, r19.reg);
				break;
			}
		default:
			break;
	}
}

void as3933_set_config()
{
}

uint8_t as3933_get_rssi(uint8_t channel)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t resp;
	switch (channel)
	{
		case 1:
			{
				r10_t r10;
				as3933_read_byte(R10, &r10.reg);
				resp = r10.rssi1;
				break;
			}
		case 2:
			{
				r11_t r11;
				as3933_read_byte(R11, &r11.reg);
				resp = r11.rssi2;
				break;
			}
		case 3:
			{
				r12_t r12;
				as3933_read_byte(R12, &r12.reg);
				resp = r12.rssi3;
				break;
			}
		default:
			resp = 0;
			break;
	}
	return resp;
}

bool as3933_get_rc_osc_calibrate_status()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r14_t r14;
	as3933_read_byte(R14, &r14.reg);
	return r14.rc_cal_ok;
}

//Enable Data slicer absolute reference
void as3933_set_data_slicer(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);
	r1.abs_hy = value;
	as3933_write_byte(R1, r1.reg);
}

//Data slicer absolute threshold reduction
void as3933_set_data_slicer_threshold_reduction(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r2_t r2;
	as3933_read_byte(R2, &r2.reg);
	r2.s_abs = value;
	as3933_write_byte(R2, r2.reg);
}

void as3933_set_disable_agc(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r9_t r9;
	//as3933_read_byte(R9, &r9.reg);
	r9.block_agc = value;
	as3933_write_byte(R9, r9.reg);
}

bool as3933_get_disable_agc(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r9_t r9;
	as3933_read_byte(R9, &r9.reg);
	return r9.block_agc;
}

void as3933_set_min_preamble_length(fs_slc_t len)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r3_t r3;
	as3933_read_byte(R3, &r3.reg);
	r3.fs_slc = len;
	as3933_write_byte(R3, r3.reg);
}

void as3933_set_listening_mode(listening_mode_t mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r0_t r0;
	as3933_read_byte(R0, &r0.reg);

	switch (mode)
	{
		case LM_STANDARD:
			r0.on_off = false;
			r0.mux_123 = false;
			break;
		case LM_SCANNING:
			r0.on_off = false;
			r0.mux_123 = true;
			break;
		case LM_ON_OFF:
			r0.on_off = true;
			r0.mux_123 = false;
			break;
		default:
			return;
	}
	as3933_write_byte(R0, r0.reg);
}

void as3933_set_freq_tolerance(s_wu1_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r2_t r2;
	as3933_read_byte(R2, &r2.reg);
	r2.s_wu1 = value;
	as3933_write_byte(R2, r2.reg);
}

void as3933_set_3db_gain_boost(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r2_t r2;
	as3933_read_byte(R2, &r2.reg);
	r2.g_boost = value;
	as3933_write_byte(R2, r2.reg);
}

void as3933_set_gain_reduction(gr_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r4_t r4;
	as3933_read_byte(R4, &r4.reg);
	r4.gr = value;
	as3933_write_byte(R4, r4.reg);
}

void as3933_enable_antenna_damper(bool value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r1_t r1;
	as3933_read_byte(R1, &r1.reg);
	r1.att_on = value;
	as3933_write_byte(R1, r1.reg);
}

void as3933_set_antenna_damper(r_val_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r4_t r4;
	as3933_read_byte(R4, &r4.reg);
	r4.r_val = value;
	as3933_write_byte(R4, r4.reg);
}

//OFF time in ON/OFF operation mode
void as3933_set_off_timer(t_off_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r4_t r4;
	as3933_read_byte(R4, &r4.reg);
	r4.t_off = value;
	as3933_write_byte(R4, r4.reg);
}

void as3933_set_comparator_hysteresis(comp_hyst_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r3_t r3;
	as3933_read_byte(R3, &r3.reg);
	r3.hy_pos = value;
	r3.hy_20m = value >> 1;
	as3933_write_byte(R3, r3.reg);
}

void as3933_set_envelop_detector(fs_env_t value)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	r3_t r3;
	as3933_read_byte(R3, &r3.reg);
	r3.fs_env = value;
	as3933_write_byte(R3, r3.reg);
}

uint8_t as3933_get_false_wake_up_register(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	r13_t r13;
	ESP_ERROR_CHECK(as3933_read_byte(R13, &r13.f_wake));
	return r13.f_wake;
}

void as3933_clear_wake_up()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_cmd(CLEAR_WAKE);
}

void as3933_clear_false_wake_up()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_cmd(CLEAR_FALSE);
}

void as3933_calibrate_rco_lc()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_cmd(CALIB_RCO_LC);
}

void as3933_reset_rssi()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	as3933_cmd(RESET_RSSI);
}

static uint16_t convert_hex_to_manchester(uint8_t hex)
{
	int8_t i = 8;
	uint16_t ret = 0;
	while (i > 0)
	{
		i--;
		ret <<= 2;
		if (hex & (1 << i))
		{
			ret |= 2;
		}
		else
		{
			ret |= 1;
		}
	}
	return ret;
}

bool as3933_rc_osc_self_calibrate()
{
	as3933_set_xtal_osc(false);
	as3933_calibrate_rco_lc();
	vTaskDelay(5000 / portTICK_PERIOD_MS);
	return as3933_get_rc_osc_calibrate_status();
}
