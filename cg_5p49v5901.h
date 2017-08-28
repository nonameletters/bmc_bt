/*
 * cg_5p49v5901.h
 *
 *  Created on: Jun 28, 2017
 *      Author: root
 */

#ifndef CG_5P49V5901_H_
#define CG_5P49V5901_H_

#include <stdint.h>

/* ********** ********** ********** ********** ********** ********** */
/* This device has only 2 addresses, check one of this               */
/* ********** ********** ********** ********** ********** ********** */
#define CG_DEFAULT_ADR  0xD4
#define CG_ADDITION_ADR 0xD0

typedef struct __cgStruct
{
	uint8_t regNum;
	uint8_t regVal;
} cgStruct;


#define REG_OTP_CONTROL                   0x00
#define REG_PRIMARY_SOURCE_SHUTDOWN       0x10
#define REG_VCO_BAND_AND_FACTORY_RESERVED 0x11
#define REG_CRYSTAL_X1_LOAD_CAPACITOR     0x12
#define REG_CRYSTAL_X2_LOAD_CAPACITOR     0x13
#define REG_REFERINCE_DRIVER              0x15
#define REG_VCO_CR_AND_PRE_DEVIDER        0x16
#define REG_FEEDBACK_INTEGER_DEVIDER      0x17
#define REG_FEEDBACK_INTEGER_DEVIDER_BIT  0x18
#define REG_FEEDBACK_FRACTIONAL_DEVIDER1  0x19
#define REG_FEEDBACK_FRACTIONAL_DEVIDER2  0x1A
#define REG_FEEDBACK_FRACTIONAL_DEVIDER3  0x1B
#define REG_RC_CONTROL1                   0x1E
#define REG_RC_CONTROL2                   0x1F

#define REG_OUT_DRIVER1_CR1_SET           0x21
#define REG_OUT_DRIVER1_FRACT_SET1        0x22
#define REG_OUT_DRIVER1_FRACT_SET2        0x23
#define REG_OUT_DRIVER1_FRACT_SET3        0x24
#define REG_OUT_DRIVER1_FRACT_SET4        0x25
#define REG_OUT_DRIVER1_SPREAD_CR1        0x26
#define REG_OUT_DRIVER1_SPREAD_CR2        0x27
#define REG_OUT_DRIVER1_SPREAD_CR3        0x28
#define REG_OUT_DRIVER1_SPREAD_MOD_R_CR1  0x29
#define REG_OUT_DRIVER1_SPREAD_MOD_R_CR2  0x2A
#define REG_OUT_DRIVER1_SKEW_INT_PART1    0x2B
#define REG_OUT_DRIVER1_SKEW_INT_PART2    0x2C
#define REG_OUT_DRIVER1_INT_PART1         0x2D
#define REG_OUT_DRIVER1_INT_PART2         0x2E
#define REG_OUT_DRIVER1_SKEW_FRACT_PART   0x2F

#define REG_OUT_DRIVER2_CR1_SET           0x31
#define REG_OUT_DRIVER2_FRACT_SET1        0x32
#define REG_OUT_DRIVER2_FRACT_SET2        0x33
#define REG_OUT_DRIVER2_FRACT_SET3        0x34
#define REG_OUT_DRIVER2_FRACT_SET4        0x35
#define REG_OUT_DRIVER2_SPREAD_CR1        0x36
#define REG_OUT_DRIVER2_SPREAD_CR2        0x37
#define REG_OUT_DRIVER2_SPREAD_CR3        0x38
#define REG_OUT_DRIVER2_SPREAD_MOD_R_CR1  0x39
#define REG_OUT_DRIVER2_SPREAD_MOD_R_CR2  0x3A
#define REG_OUT_DRIVER2_SKEW_INT_PART1    0x3B
#define REG_OUT_DRIVER2_SKEW_INT_PART2    0x3C
#define REG_OUT_DRIVER2_INT_PART1         0x3D
#define REG_OUT_DRIVER2_INT_PART2         0x3E
#define REG_OUT_DRIVER2_SKEW_FRACT_PART   0x3F

#define REG_OUT_DRIVER3_CR1_SET           0x41
#define REG_OUT_DRIVER3_FRACT_SET1        0x42
#define REG_OUT_DRIVER3_FRACT_SET2        0x43
#define REG_OUT_DRIVER3_FRACT_SET3        0x44
#define REG_OUT_DRIVER3_FRACT_SET4        0x45
#define REG_OUT_DRIVER3_SPREAD_CR1        0x46
#define REG_OUT_DRIVER3_SPREAD_CR2        0x47
#define REG_OUT_DRIVER3_SPREAD_CR3        0x48
#define REG_OUT_DRIVER3_SPREAD_MOD_R_CR1  0x49
#define REG_OUT_DRIVER3_SPREAD_MOD_R_CR2  0x4A
#define REG_OUT_DRIVER3_SKEW_INT_PART1    0x4B
#define REG_OUT_DRIVER3_SKEW_INT_PART2    0x4C
#define REG_OUT_DRIVER3_INT_PART1         0x4D
#define REG_OUT_DRIVER3_INT_PART2         0x4E
#define REG_OUT_DRIVER3_SKEW_FRACT_PART   0x4F

#define REG_OUT_DRIVER4_CR1_SET           0x51
#define REG_OUT_DRIVER4_FRACT_SET1        0x52
#define REG_OUT_DRIVER4_FRACT_SET2        0x53
#define REG_OUT_DRIVER4_FRACT_SET3        0x54
#define REG_OUT_DRIVER4_FRACT_SET4        0x55
#define REG_OUT_DRIVER4_SPREAD_CR1        0x56
#define REG_OUT_DRIVER4_SPREAD_CR2        0x57
#define REG_OUT_DRIVER4_SPREAD_CR3        0x58
#define REG_OUT_DRIVER4_SPREAD_MOD_R_CR1  0x59
#define REG_OUT_DRIVER4_SPREAD_MOD_R_CR2  0x5A
#define REG_OUT_DRIVER4_SKEW_INT_PART1    0x5B
#define REG_OUT_DRIVER4_SKEW_INT_PART2    0x5C
#define REG_OUT_DRIVER4_INT_PART1         0x5D
#define REG_OUT_DRIVER4_INT_PART2         0x5E
#define REG_OUT_DRIVER4_SKEW_FRACT_PART   0x5F

#define REG_OUT_CLOCK1_CONF1              0x60
#define REG_OUT_CLOCK1_CONF2              0x62
#define REG_OUT_CLOCK1_CONF3              0x64
#define REG_OUT_CLOCK1_CONF4              0x66
#define REG_OUT_CLOCK2_CONF1              0x61
#define REG_OUT_CLOCK2_CONF2              0x63
#define REG_OUT_CLOCK2_CONF3              0x65
#define REG_OUT_CLOCK2_CONF4              0x67
#define REG_CLK_OE_SD_FUNC1               0x68
#define REG_CLK_OE_SD_FUNC2               0x69

/* ********** ********** ********** ********** ********** ********** ********** ********** ********** */
/* This is configuration is generated in Timing Commander                                             */
/* https://www.idt.com/products/clocks-timing/timing-commander-software-download-resource-guide       */
/* ********** ********** ********** ********** ********** ********** ********** ********** ********** */
// I didn't yet copy values to table
cgStruct defaultConf[] = { {REG_OTP_CONTROL,                   0x00}, \
                           {REG_PRIMARY_SOURCE_SHUTDOWN,       0x10}, \
                           {REG_VCO_BAND_AND_FACTORY_RESERVED, 0x11}, \
                           {REG_CRYSTAL_X1_LOAD_CAPACITOR,     0x12}, \
                           {REG_CRYSTAL_X2_LOAD_CAPACITOR,     0x13}, \
                           {REG_REFERINCE_DRIVER,              0x15}, \
                           {REG_VCO_CR_AND_PRE_DEVIDER,        0x16}, \
                           {REG_FEEDBACK_INTEGER_DEVIDER,      0x17}, \
                           {REG_FEEDBACK_INTEGER_DEVIDER_BIT,  0x18}, \
                           {REG_FEEDBACK_FRACTIONAL_DEVIDER1,  0x19}, \
                           {REG_FEEDBACK_FRACTIONAL_DEVIDER2,  0x1A}, \
                           {REG_FEEDBACK_FRACTIONAL_DEVIDER3,  0x1B}, \
                           {REG_RC_CONTROL1,                   0x1E}, \
                           {REG_RC_CONTROL2,                   0x1F}, \
                                                                      \
                           {REG_OUT_DRIVER1_CR1_SET,           0x21}, \
                           {REG_OUT_DRIVER1_FRACT_SET1,        0x22}, \
                           {REG_OUT_DRIVER1_FRACT_SET2,        0x23}, \
                           {REG_OUT_DRIVER1_FRACT_SET3,        0x24}, \
                           {REG_OUT_DRIVER1_FRACT_SET4,        0x25}, \
                           {REG_OUT_DRIVER1_SPREAD_CR1,        0x26}, \
                           {REG_OUT_DRIVER1_SPREAD_CR2,        0x27}, \
                           {REG_OUT_DRIVER1_SPREAD_CR3,        0x28}, \
                           {REG_OUT_DRIVER1_SPREAD_MOD_R_CR1,  0x29}, \
                           {REG_OUT_DRIVER1_SPREAD_MOD_R_CR2,  0x2A}, \
                           {REG_OUT_DRIVER1_SKEW_INT_PART1,    0x2B}, \
                           {REG_OUT_DRIVER1_SKEW_INT_PART2,    0x2C}, \
                           {REG_OUT_DRIVER1_INT_PART1,         0x2D}, \
                           {REG_OUT_DRIVER1_INT_PART2,         0x2E}, \
                           {REG_OUT_DRIVER1_SKEW_FRACT_PART,   0x2F}, \
                                                                      \
						   {REG_OUT_DRIVER2_CR1_SET,           0x31}, \
                           {REG_OUT_DRIVER2_FRACT_SET1,        0x32}, \
                           {REG_OUT_DRIVER2_FRACT_SET2,        0x33}, \
                           {REG_OUT_DRIVER2_FRACT_SET3,        0x34}, \
                           {REG_OUT_DRIVER2_FRACT_SET4,        0x35}, \
                           {REG_OUT_DRIVER2_SPREAD_CR1,        0x36}, \
                           {REG_OUT_DRIVER2_SPREAD_CR2,        0x37}, \
                           {REG_OUT_DRIVER2_SPREAD_CR3,        0x38}, \
                           {REG_OUT_DRIVER2_SPREAD_MOD_R_CR1,  0x39}, \
                           {REG_OUT_DRIVER2_SPREAD_MOD_R_CR2,  0x3A}, \
                           {REG_OUT_DRIVER2_SKEW_INT_PART1,    0x3B}, \
                           {REG_OUT_DRIVER2_SKEW_INT_PART2,    0x3C}, \
                           {REG_OUT_DRIVER2_INT_PART1,         0x3D}, \
                           {REG_OUT_DRIVER2_INT_PART2,         0x3E}, \
                           {REG_OUT_DRIVER2_SKEW_FRACT_PART,   0x3F}, \
                                                                      \
		                   {REG_OUT_DRIVER3_CR1_SET,           0x41}, \
                           {REG_OUT_DRIVER3_FRACT_SET1,        0x42}, \
                           {REG_OUT_DRIVER3_FRACT_SET2,        0x43}, \
                           {REG_OUT_DRIVER3_FRACT_SET3,        0x44}, \
                           {REG_OUT_DRIVER3_FRACT_SET4,        0x45}, \
                           {REG_OUT_DRIVER3_SPREAD_CR1,        0x46}, \
                           {REG_OUT_DRIVER3_SPREAD_CR2,        0x47}, \
                           {REG_OUT_DRIVER3_SPREAD_CR3,        0x48}, \
                           {REG_OUT_DRIVER3_SPREAD_MOD_R_CR1,  0x49}, \
                           {REG_OUT_DRIVER3_SPREAD_MOD_R_CR2,  0x4A}, \
                           {REG_OUT_DRIVER3_SKEW_INT_PART1,    0x4B}, \
                           {REG_OUT_DRIVER3_SKEW_INT_PART2,    0x4C}, \
                           {REG_OUT_DRIVER3_INT_PART1,         0x4D}, \
                           {REG_OUT_DRIVER3_INT_PART2,         0x4E}, \
                           {REG_OUT_DRIVER3_SKEW_FRACT_PART,   0x4F}, \
                                                                      \
				           {REG_OUT_DRIVER4_CR1_SET,           0x51}, \
                           {REG_OUT_DRIVER4_FRACT_SET1,        0x52}, \
                           {REG_OUT_DRIVER4_FRACT_SET2,        0x53}, \
                           {REG_OUT_DRIVER4_FRACT_SET3,        0x54}, \
                           {REG_OUT_DRIVER4_FRACT_SET4,        0x55}, \
                           {REG_OUT_DRIVER4_SPREAD_CR1,        0x56}, \
                           {REG_OUT_DRIVER4_SPREAD_CR2,        0x57}, \
                           {REG_OUT_DRIVER4_SPREAD_CR3,        0x58}, \
                           {REG_OUT_DRIVER4_SPREAD_MOD_R_CR1,  0x59}, \
                           {REG_OUT_DRIVER4_SPREAD_MOD_R_CR2,  0x5A}, \
                           {REG_OUT_DRIVER4_SKEW_INT_PART1,    0x5B}, \
                           {REG_OUT_DRIVER4_SKEW_INT_PART2,    0x5C}, \
                           {REG_OUT_DRIVER4_INT_PART1,         0x5D}, \
                           {REG_OUT_DRIVER4_INT_PART2,         0x5E}, \
                           {REG_OUT_DRIVER4_SKEW_FRACT_PART,   0x5F}, \
                                                                      \
		                   {REG_OUT_CLOCK1_CONF1,              0x60}, \
                           {REG_OUT_CLOCK1_CONF2,              0x62}, \
                           {REG_OUT_CLOCK1_CONF3,              0x64}, \
                           {REG_OUT_CLOCK1_CONF4,              0x66}, \
                           {REG_OUT_CLOCK2_CONF1,              0x61}, \
                           {REG_OUT_CLOCK2_CONF2,              0x63}, \
                           {REG_OUT_CLOCK2_CONF3,              0x65}, \
                           {REG_OUT_CLOCK2_CONF4,              0x67}, \
                           {REG_CLK_OE_SD_FUNC1,               0x68}, \
                           {REG_CLK_OE_SD_FUNC2,               0x69}  \
                         };



#endif /* CG_5P49V5901_H_ */
