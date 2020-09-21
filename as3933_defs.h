#ifndef AS3933_DEFS_H
#define AS3933_DEFS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      DEFINES
 *********************/

#define R0 		0x00
#define R1 		0x01
#define R2 		0x02
#define R3 		0x03
#define R4 		0x04
#define R5 		0x05
#define R6 		0x06
#define R7 		0x07
#define R8  	0x08
#define R9 		0x09
#define R10 	0x0A
#define R11		0x0B
#define R12 	0x0C
#define R13 	0x0D
#define R14 	0x0E
#define R15 	0x0F
#define R16 	0x10
#define R17 	0x11
#define R18 	0x12
#define R19 	0x13

#define AS3933_WRITE_SINGLE 		0x00	/*!<  WRITE                 */
#define AS3933_READ_SINGLE 			0x01	/*!<  READ                  */
#define AS3933_DIRECT_COMMAND 		0x03	/*!<  DIRECT COMMAND        */

/**********************
 *      MACROS
 **********************/
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c %c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
	(byte & 0x80 ? '1' : '0'), \
	(byte & 0x40 ? '1' : '0'), \
	(byte & 0x20 ? '1' : '0'), \
	(byte & 0x10 ? '1' : '0'), \
	(byte & 0x08 ? '1' : '0'), \
	(byte & 0x04 ? '1' : '0'), \
	(byte & 0x02 ? '1' : '0'), \
	(byte & 0x01 ? '1' : '0')

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AS3933_DEFS_H */
