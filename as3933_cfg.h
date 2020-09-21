/*
 * as3933_cfg.h
 *
 *  Created on: 12 cze 2020
 *      Author: kurza
 */

#ifndef AS3933_CFG_H_
#define AS3933_CFG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "as3933_defs.h"
#include "as3933_regs.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  GLOBAL VARIABLES
 **********************/
// @formatter:off
static as3933_reg_setting_t as3933_config_register[] =
	{
			{ R0, 0b00001000 },
			// Correlator disable, Antena dumper (R1.4) enable, AGC up&down
			{ R1, 0b00110001 },
			//R2.5 - 3dB gain boost, R2.[1:0] LOOSE freq tolerance
			{ R2, 0b00000000 },
			//Hyst R3.[7:6] 00 - 40mV,  10 - 20mV
			{ R3, 0b00000000 },
			//Antena dumper r4.[5:4] 00 - 1kOm, 01 - 3kOm, 10 - 9kOm, 11 - 27 kOm
			//Gain reduction R4.[3:0]
			{ R4, 0b00000000 },
			//R7[5:7] Timeout
			{ R7, 0b11101011 }, // timeout on
			//{R7, 0b00001011}, // timeout off
			//			{ R8, 0b00000000 },
			//AGC enable
			//			{ R9, 0b00000000 },
			//			{ R16, 0b00000000 },
			//Capacitors setting
			{ R17, 0b00011111 },
			{ R18, 0b00011111 },
			{ R19, 0b00011111 }
	};
// @formatter:on
/*
{ R0, 0b00001110 },
// Correlator disable, Antena dumper (R1.4) enable, AGC up&down
{ R1, 0b00110001 },
//R2.5 - 3dB gain boost, R2.[1:0] LOOSE freq tolerance
{ R2, 0b00000000 },
//Hyst R3.[7:6] 00 - 40mV,  10 - 20mV
{ R3, 0b00000000 },
//Antena dumper r4.[5:4] 00 - 1kOm, 01 - 3kOm, 10 - 9kOm, 11 - 27 kOm
//Gain reduction R4.[3:0]
{ R4, 0b00001110 },
//R7[5:7] Timeout
{ R7, 0b11101011 }, // timeout on
//{R7, 0b00001011}, // timeout off
//			{ R8, 0b00000000 },
//AGC enable
//			{ R9, 0b00000000 },
//			{ R16, 0b00000000 },
//Capacitors setting
//			{ R17, 0b00000101 },
//			{ R18, 0b00001111 },
//			{ R19, 0b00010001 }
*/
//static as3933_init_cmd_t as3933_default_register[] = {
//	{R0, 0b00001110},
//	{R1, 0b00100011},
//	{R2, 0b00000000},
//	{R3, 0b00100000},
//	{R4, 0b00010000},
//	{R5, 0b01101001},
//	{R6, 0b10010110},
//	{R7, 0b00001011},
//	{R8, 0b00000000},
//	{R9, 0b00000000},
//	{R16, 0b00000000},
//	{R17, 0b00000101},
//	{R18, 0b00001111},
//	{R19, 0b00010001}};
// {R17, 0b00000000},
// {R18, 0b00000000},
// {R19, 0b00000000}};


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_CFG_H_ */
