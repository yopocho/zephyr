/**
 * @file    ieee802154_s2-lp_qi.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2LP QI.
 * @details
 *
 * This module can be used to configure and read some quality indicators
 * used by S2-LP.
 * API to set thresholds and to read values in raw mode or in dBm are
 * provided.
 *
 * <b>Example:</b>
 * @code
 *
 *   int32_t rssiValuedBm;
 *   uint8_t pqiValue, sqiValue;
 *
 *   S2LPQiPqiCheck(S_ENABLE);
 *   S2LPQiSqiCheck(S_ENABLE);
 *
 *   ...
 *
 *   rssiValueDbm = S2LPQiGetRssidBm();
 *   pqiValue = S2LPQiGetPqi();
 *   sqiValue = S2LPQiGetSqi();
 *
 *   ...
 *
 * @endcode
 */

#ifndef IEEE802154_S2LP_QI_H_
#define IEEE802154_S2LP_QI_H_

#include "ieee802154_s2lp_registers.h"
#include "ieee802154_s2lp_types.h"
#include "ieee802154_s2lp_core_spi.h"

/**
 * @brief  S2LP RSSI mode enumeration
 */
typedef enum {
  RSSI_STATIC_MODE = 0,         /* static mode */
  RSSI_DYNAMIC_6DB_STEP_MODE,   /* dynamic mode 6 dB above threshold*/
  RSSI_DYNAMIC_12DB_STEP_MODE,  /* dynamic mode 12 dB above threshold */
  RSSI_DYNAMIC_18DB_STEP_MODE   /* dynamic mode 18 dB above threshold */
} SRssiMode;

/**
 * @brief  S2LP RSSI Init structure definition
 */
typedef struct {
  uint8_t      cRssiFlt;       /*!< Set the RSSI filter gain. From 0 to 15. */
  SRssiMode    xRssiMode;      /*!< Set the RSSI mode. @ref SRssiMode */
  int32_t        cRssiThreshdBm; /*!< Set the RSSI threshold in dBm. From -130 to -2.5 dBm. */
} SRssiInit;

int32_t S2LPRadioGetRssidBm(void);
int32_t S2LPRadioGetRssidBmRun(void);
void S2LPRadioSetRssiThreshdBm(int32_t wRssiThrehsold);
void S2LPRadioCsBlanking(SFunctionalState xCsBlank);
void S2LPRadioRssiInit(SRssiInit* xSRssiInit);
void S2LPRadioGetRssiInfo(SRssiInit* xSRssiInit);
void S2LPRadioAntennaSwitching(SFunctionalState xAntennaSwitch);
void S2LPRadioSetPQIThreshold(uint8_t cPQIThreshold);
void S2LPRadioSetSQIThreshold(uint8_t cSQIThreshold);
void S2LPRadioEnableSQI(SFunctionalState xSQIEnable);
SFlagStatus S2LPQiGetCs(void);

#endif 