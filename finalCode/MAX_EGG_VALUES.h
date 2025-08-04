#ifndef MAX_EGG_VALUES_H
#define MAX_EGG_VALUES_H

// ************************************************************
// MAX DEFINED REGISTER VALUES
// ************************************************************
// Register values for configuring the MAX to enable EGG measurements
// by outputting a 128 kHz square wave

// Some values from BIOZ header are also needed for functions
#include "MAX_BIOZ_VALUES.h"

// ************************************************************
// EN_INT / EN_INT2
// ************************************************************
// Use disabled registers from BIOZ_VALUES

// ************************************************************
// MNGR_INT
// ************************************************************
// Use MNGR_INT from BIOZ_VALUES

// ************************************************************
// SYNCH
// ************************************************************
//

// ************************************************************
// CNFG_GEN
// ************************************************************
#define CNFG_GEN_EGG 0x00140027
// D[23:22] EN_ULP_LON  = 00 for ULP lead-on detection disabled
// D[21:20] FMSTR       = 01 for 32 kHz FMSTR
// D[19]    EN_ECG      = 0 for ECG Channel disabled
// D[18]    EN_BIOZ     = 1 for BIOZ Channel enabled
// D[17]    EN_PACE     = 0 for PACE channel disabled
// D[15:14] EN_BLOFF    = 00 for BioZ lead-off disabled
// D[13:12] EN_DCLOFF   = 00 for DC Lead-off detection disabled
// D[11]    DCLOFF_IPOL = 0 for DC lead-off current polarity
// D[10:8]  IMAG        = 00 for 0 nA DC Lead-Off current magnitude
// D[7:6]   VTH         = 00 DC Lead-Off Voltage Threshold
// D[5:4]   EN_RBIAS    = 10 for BioZ resistive bias enabled
// D[3:2]   RBIASV      = 01 for 100M resistive bias
// D[1]     RBIASP      = 1 for resistive bias on positive input
// D[0]     RBIASN      = 1 for resistive bias on negative input

// ************************************************************
// CNFG_BMUX
// ************************************************************

// ************************************************************
// CNFG_BIOZ
// ************************************************************
#define CNFG_BIOZ_EGG 0x00670070
// Configure BIOZ channel for outputting 128 kHz square wave

// D[23]    BIOZ_RATE   = 0 for higher sample rate option
// D[22:20] BIOZ_AHPF   = 110 for bypass analog HPF
// D[19]    EXT_RBIAS   = 0 for internal bias generator
// D[18]    LN_BIOZ     = 1 for low-noise mode
// D[17:16] BIOZ_GAIN   = 11 for 80 V/V
// D[15:14] BIOZ_DHPF   = 00 for bypass of HPF
// D[13:12] BIOZ_DLPF   = 00 for bypass of LPF
// D[11:8]  BIOZ_FCGEN  = 0000 for default frequency of 128 kHz.  This makes applying mask much simpler
// D[7]     BIOZ_CGMON  = 0 for disabled monitoring of current generator
// D[6:4]   BIOZ_CGMAG  = 111 for 96uA current generator magnitude
// D[3:0]   BIOZ_PHOFF  = 0000 for 0* phase offset

#endif