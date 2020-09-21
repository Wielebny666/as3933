#ifndef AS3933_H
#define AS3933_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "as3933_regs.h"

/*********************
 *      DEFINES
 *********************/
#define XTAL_FREQ 32000


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void as3933_init_config(void);
void as3933_read_config(void);
void as3933_print_config(void);

void as3933_reset();

void as3933_set_channel(uint8_t channel, bool value);
bool as3933_get_channel(uint8_t channel);

void as3933_set_mask_data_before_wu(bool select);
void as3933_set_manchaster_decode(bool select);
void as3933_set_patern_correlation(wake_up_mode_t mode);
void as3933_set_wakeup_pattern_16bit(uint16_t wakeup_node_id);
uint16_t as3933_get_wakeup_pattern_16bit();
void as3933_set_wakeup_pattern_8bit(uint8_t wakeup_node_id);
void as3933_set_bitrate(uint8_t value);
void as3933_set_auto_time_out(t_out_t value);
void as3933_band_select(band_sel_t freq);
void as3933_route_res_freq_on_dat(uint8_t channel, bool value);
void as3933_route_clock_on_dat(bool value);
void as3933_set_xtal_osc(bool value);
void as3933_set_agc_up_and_down(bool value);
void as3933_set_agc_first_carrier(bool value);
// 0 - 31 pF
void as3933_set_capacity(uint8_t channel, uint8_t value);
uint8_t as3933_get_rssi(uint8_t channel);
bool as3933_get_rc_osc_calibrate_status();
void as3933_set_data_slicer(bool value);
void as3933_set_data_slicer_threshold_reduction(bool value);

void as3933_set_disable_agc(bool value);
bool as3933_get_disable_agc(void);
void as3933_set_min_preamble_length(fs_slc_t len);
void as3933_set_listening_mode(listening_mode_t mode);
void as3933_set_freq_tolerance(s_wu1_t value);
void as3933_set_gain_reduction(gr_t value);
void as3933_enable_antenna_damper(bool value);
void as3933_set_antenna_damper(r_val_t value);
void as3933_set_off_timer(t_off_t value);
void as3933_set_comparator_hysteresis(comp_hyst_t value);
void as3933_set_envelop_detector(fs_env_t value);
void as3933_set_3db_gain_boost(bool value);
uint8_t as3933_get_false_wake_up_register(void);

//commands
void as3933_clear_wake_up();
void as3933_clear_false_wake_up();
void as3933_calibrate_rco_lc();
void as3933_reset_rssi();

bool as3933_rc_osc_self_calibrate();


/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_H */
