/**
 * @file    S2LP_General.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP General functionalities.
 */

#ifndef IEEE802154_S2LP_GENERAL_H_
#define IEEE802154_S2LP_GENERAL_H_

#include "ieee802154_s2lp_registers.h"
#include "ieee802154_s2lp_types.h"
#include "ieee802154_s2lp_core_spi.h"

/**
 * @brief  S2LP version type enumeration
 */

typedef enum {
  MODE_EXT_XO  = 0,
  MODE_EXT_XIN = 0x80,
} ModeExtRef;

#define S2LPGeneralLibraryVersion() "S2LP_Libraries_v.1.3.0"

uint8_t S2LPGeneralGetDevicePN(void);
uint8_t S2LPGeneralGetVersion(void);
void S2LPGeneralSetExtRef(ModeExtRef xExtMode);
ModeExtRef S2LPGeneralGetExtRef(void);
void S2LPRadioSetExternalSmpsMode(SFunctionalState xNewState);
void S2LPRefreshStatus(void);


#endif