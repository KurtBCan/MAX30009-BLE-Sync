#ifndef MAX_BIOZ_VALUES_H
#define MAX_BIOZ_VALUES_H

// ************************************************************
// MAX 30009 DEFINED REGISTER VALUES
// ************************************************************
// Various default and configured register values for
// capturing BioZ measurements with the MAX 30009
// Default values for MAX 30009 registers
//
// 


#define SYS_CONFIG_1 0x02

#define PIN_FUNC_CONFIG 0x04

#define PLL_CONFIG_1 0x53
#define PLL_CONFIG_2 0xFF
#define PLL_CONFIG_3 0x01
#define PLL_CONFIG_4 0x60

#define BIOZ_CONFIG_1 0xF5
#define BIOZ_CONFIG_2 0x20
#define BIOZ_CONFIG_3 0x28
#define BIOZ_CONFIG_4 0x00
#define BIOZ_CONFIG_5 0x33
#define BIOZ_CONFIG_6 0xCA
#define BIOZ_LW_THRSHLD 0x00
#define BIOZ_HI_THRSHLD 0xFF
#define BIOZ_CONFIG_7 0x02

#define BIOZ_MUX_CONFIG_1 0x06
#define BIOZ_MUX_CONFIG_2 0x01
#define BIOZ_MUX_CONFIG_3 0xA0
#define BIOZ_MUX_CONFIG_4 0xF0

#define DC_LEADS_CONFIG 0x00
#define DC_LEAD_DTECT_THRSHLD 0x00

#define LEAD_BIAS_CONFIG_1 0x07

#define INT_EN_1 0x80
#define INT_EN_2 0x00

#define PART_ID 0x42


#endif
