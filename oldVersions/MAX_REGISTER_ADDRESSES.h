#ifndef MAX_REGISTER_ADDRESSES_H
#define MAX_REGISTER_ADDRESSES_H

// Register addresses for the MAX30009

// ************************************************************
// MAX DEFINED REGISTER Addresses
// ************************************************************
// Macros for register addresses used by BioZ configuration of MAX


#define DONT_CARE 0xFF
#define WRITE_BYTE 0x00
#define READ_BYTE 0x80


// Status registers
#define REG_STATUS1 0x00 // Read ONLY
#define REG_STATUS2 0x01 // Read ONLY

// FIFO Registers
#define REG_FIFO_W_PTR 0x08 // Read ONLY
#define REG_FIFO_R_PTR 0x09
#define REG_FIFO_CTR_1 0x0A // Read ONLY
#define REG_FIFO_CTR_2 0x0B // Read ONLY
#define REG_FIFO_DATA  0x0C // Read ONLY
#define REG_FIFO_CONFIG_1 0x0D
#define REG_FIFO_CONFIG_2 0x0E

// System Control Registers
#define REG_SYS_SYNC 0x10
#define REG_SYS_CONFIG_1 0x11
#define REG_PIN_FUNC_CONFIG 0x12
#define REG_OUT_PIN_CONFIG 0x13
#define REG_I2C_BRDCST_ADDR 0x14

// PLL Registers
#define REG_PLL_CONFIG_1 0x17
#define REG_PLL_CONFIG_2 0x18
#define REG_PLL_CONFIG_3 0x19
#define REG_PLL_CONFIG_4 0x1A

// BioZ Setup Registers
#define REG_BIOZ_CONFIG_1 0x20
#define REG_BIOZ_CONFIG_2 0x21
#define REG_BIOZ_CONFIG_3 0x22
#define REG_BIOZ_CONFIG_4 0x23
#define REG_BIOZ_CONFIG_5 0x24
#define REG_BIOZ_CONFIG_6 0x25
#define REG_BIOZ_LW_THRSHLD 0x26
#define REG_BIOZ_HI_THRSHLD 0x27
#define REG_BIOZ_CONFIG_7 0x28

// BioZ Calibration Registers
#define REG_BIOZ_MUX_CONFIG_1 0x41
#define REG_BIOZ_MUX_CONFIG_2 0x42
#define REG_BIOZ_MUX_CONFIG_3 0x43
#define REG_BIOZ_MUX_CONFIG_4 0x44

// DC Leads Setup Registers
#define REG_DCLEADS_CONFIG 0x50
#define REG_DCLEAD_DTECT_THRSHLD 0x51

// Lead Bias Register
#define REG_LEAD_BIAS_CONFIG_1 0x58

// Interrupt Enbale Registers
#define REG_INT_EN_1 0x80
#define REG_INT_EN_2 0x81

// Part ID Register
#define REG_PART_ID 0xFF // Read ONLY





// // MAX30001 Register Addresses
// // Status and operational registers

// #define REG_NOOP1 0x00
// #define REG_STATUS 0x01
// #define REG_INT1 0x02
// #define REG_INT2 0x03
// #define REG_MNGR_INT 0x04
// #define REG_MNGR_DYN 0x05
// #define REG_SW_RST 0x08
// #define REG_SYNCH 0x09
// #define REG_INFO 0x0F
// #define REG_NOOP2 0x7F

// // Configuration registers

// #define REG_CNFG_GEN 0x10
// #define REG_CNFG_CAL 0x12
// #define REG_CNFG_EMUX 0x14
// #define REG_CNFG_ECG 0x15
// #define REG_CNFG_BMUX 0x17
// #define REG_CNFG_BIOZ 0x18
// #define REG_CNFG_PACE 0x1A
// #define REG_CNFG_RTOR1 0x1D
// #define REG_CNFG_RTOR2 0x1E

// // FIFO Registers

// #define REG_ECG_FIFO_BURST 0x20
// #define REG_ECG_FIFO_NORMAL 0x21
// #define REG_BIOZ_FIFO_BURST 0x22
// #define REG_BIOZ_FIFO_NORMAL 0x23
// #define REG_RTOR 0x25

#endif
