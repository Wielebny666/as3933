/*
 * as3933_enums.h
 *
 *  Created on: 1 cze 2020
 *      Author: kurza
 */

#ifndef AS3933_ENUMS_H
#define AS3933_ENUMS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
	CLEAR_WAKE = 0,
	RESET_RSSI = 1,
	CALIB_RCOSC = 2,
	CLEAR_FALSE = 3,
	PRESET_DEFAULT = 4,
	CALIB_RCO_LC = 5
} cmd_t;

typedef enum
{
	TOLERANCE_LOOSE,
	TOLERANCE_NORMAL,
	TOLERANCE_TIGHT
} s_wu1_t;

typedef enum
{
	PREAMBLE_LEN_0_8MS = 0,
	PREAMBLE_LEN_1_15MS,
	PREAMBLE_LEN_1_55MS,
	PREAMBLE_LEN_1_9MS,
	PREAMBLE_LEN_2_3MS,
	PREAMBLE_LEN_2_65MS,
	PREAMBLE_LEN_3_0MS,
	PREAMBLE_LEN_3_5MS
} fs_slc_t;

typedef enum
{
	EDGE_BOTH_40MV = 0,
	EDGE_POSITIVE_40MV,
	EDGE_BOTH_20MV,
	EDGE_POSITIVE_20MV
} comp_hyst_t;

typedef enum
{
	SYMBOL_RATE_4096,
	SYMBOL_RATE_2184,
	SYMBOL_RATE_1490,
	SYMBOL_RATE_1130,
	SYMBOL_RATE_910,
	SYMBOL_RATE_762,
	SYMBOL_RATE_655,
	SYMBOL_RATE_512
} fs_env_t;

typedef enum
{
	NO_GAIN = 0,
	GAIN_REDUCTION_4DB = 0b00000100,
	GAIN_REDUCTION_8DB = 0b00000110,
	GAIN_REDUCTION_12DB = 0b00001000,
	GAIN_REDUCTION_16DB = 0b00001010,
	GAIN_REDUCTION_20DB = 0b00001100,
	GAIN_REDUCTION_24DB = 0b00001110
} gr_t;

typedef enum
{
	RESISTOR_1KOM,
	RESISTOR_3KOM,
	RESISTOR_9KOM,
	RESISTOR_27KOM
} r_val_t;

typedef enum
{
	OFF_1MS,
	OFF_2MS,
	OFF_4MS,
	OFF_8MS
} t_off_t;

typedef enum
{
	TIMEOUT_DISABLE,
	TIMEOUT_50MS,
	TIMEOUT_100MS,
	TIMEOUT_150MS,
	TIMEOUT_200MS,
	TIMEOUT_250MS,
	TIMEOUT_300MS,
	TIMEOUT_350MS
} t_out_t;

typedef enum
{
	BAND_95_150_KHZ = 0,
	BAND_65_95_KHZ = 1,
	BAND_40_65_KHZ = 2,
	BAND_23_40_KHZ = 3,
	BAND_15_23_KHZ = 7
} band_sel_t;

typedef enum
{
	LM_STANDARD,
	LM_SCANNING,
	LM_ON_OFF
} listening_mode_t;

typedef enum
{
	WK_FREQ_DET_ONLY,
	WK_SINGLE_PATTERN
} wake_up_mode_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_ENUMS_H */
