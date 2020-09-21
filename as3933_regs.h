#ifndef AS3933_REGS_H
#define AS3933_REGS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "esp_types.h"
#include "as3933_enums.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
		uint8_t addr;
		uint8_t data;
} as3933_reg_setting_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t unused :1;
				uint8_t en_a1 :1;
				uint8_t en_a3 :1;
				uint8_t en_a2 :1;
				uint8_t mux_123 :1;
				uint8_t on_off :1;
				uint8_t dat_mask :1;
				enum
				{
					PAT16, PAT32
				} patt :1;
		};
		uint8_t reg;
} r0_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t en_xtal :1;
				uint8_t en_wpat :1;
				uint8_t en_pat2 :1;
				uint8_t en_manch :1;
				uint8_t att_on :1;
				uint8_t agc_ud :1;
				uint8_t agc_tlim :1;
				uint8_t abs_hy :1;
		};
		uint8_t reg;
} r1_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				s_wu1_t s_wu1 :2;
				uint8_t display_clk :2;
				uint8_t reserved :1;
				uint8_t g_boost :1;
				uint8_t en_ext_clk :1;
				uint8_t s_abs :1;
		};
		uint8_t reg;
} r2_t;



typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				fs_env_t fs_env :3;
				fs_slc_t fs_slc :3;
				uint8_t hy_pos :1;
				uint8_t hy_20m :1;
		};
		uint8_t reg;
} r3_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				gr_t gr :4;
				r_val_t r_val :2;
				t_off_t t_off :2;
		};
		uint8_t reg;
} r4_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
		uint8_t patt2b;
} r5_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
		uint8_t patt1b;
} r6_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				r6_t r6;
				r5_t r5;
		};
		uint16_t reg;
} patt_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t t_hbit :5;
				t_out_t t_out :3;
		};
		uint8_t reg;
} r7_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t t_auto :3;
				uint8_t reserved :2;
				band_sel_t band_sel :3;
		};
		uint8_t reg;
} r8_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t reserved :7;
				uint8_t block_agc :1;
		};
		uint8_t reg;
} r9_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t rssi1 :5;
				uint8_t reserved :3;
		};
		uint8_t reg;
} r10_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t rssi2 :5;
				uint8_t reserved :3;
		};
		uint8_t reg;
} r11_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t rssi3 :5;
				uint8_t reserved :3;
		};
		uint8_t reg;
} r12_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
		uint8_t f_wake;
} r13_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t rc_osc_taps :6;
				uint8_t rc_cal_ko :1;
				uint8_t rc_cal_ok :1;
		};
		uint8_t reg;
} r14_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t reserved :3;
				uint8_t lc_osc_ko :1;
				uint8_t lc_osc_ok :1;
				uint8_t reserved_ :3;
		};
		uint8_t reg;
} r15_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t lc_osc_mux1 :1;
				uint8_t lc_osc_mux2 :1;
				uint8_t lc_osc_mux3 :1;
				uint8_t n_a_4 :1;
				uint8_t rc_osc_max :1;
				uint8_t rc_osc_min :1;
				uint8_t n_a_6 :1;
				uint8_t clock_gen_dis :1;
		};
		uint8_t reg;
} r16_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t cap_ch1 :5;
				uint8_t n_a_4 :3;
		};
		uint8_t reg;
} r17_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t cap_ch2 :5;
				uint8_t n_a_4 :3;
		};
		uint8_t reg;
} r18_t;

typedef union __attribute__((__packed__, aligned(1)))
{
		struct __attribute__((__packed__, aligned(1)))
		{
				uint8_t cap_ch3 :5;
				uint8_t n_a_4 :3;
		};
		uint8_t reg;
} r19_t;

typedef struct __attribute__((__packed__, aligned(1))) as3933_t
{
		r0_t r0;
		r1_t r1;
		r2_t r2;
		r3_t r3;
		r4_t r4;
		r5_t r5;
		r6_t r6;
		r7_t r7;
		r8_t r8;
		r9_t r9;
		r10_t r10;
		r11_t r11;
		r12_t r12;
		r13_t r13;
		r14_t r14;
		r15_t r15;
		r16_t r16;
		r17_t r17;
		r18_t r18;
		r19_t r19;
} as3933_t;

extern as3933_t as3933;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_REGS_H */
