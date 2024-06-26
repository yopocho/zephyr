/**
 * @file    S2LP_Csma.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP CSMA.
 * @details
 *
 * The S2LP CSMA feature, when configured and enabled, is transparent
 * for the user. It means the user has only to call the <i>@ref S2LPCsmaInit()</i>
 * function on a filled structure and then enable the CSMA policy using the <i>@ref S2LPCsma()</i>
 * function.
 *
 * <b>Example:</b>
 * @code
 *
 * CsmaInit csmaInit={
 *   S_DISABLE,         // persistent mode
 *   TBIT_TIME_64,      // Tbit time
 *   TCCA_TIME_3,       // Tcca time
 *   5,                 // max number of backoffs
 *   0xFA21,            // BU counter seed
 *   32                 // CU prescaler
 * };
 *
 * ...
 *
 * S2LPCsmaInit(&csmaInit);
 * S2LPCsma(S_ENABLE);
 *
 *
 * @endcode
 *
 * @note The CS status depends of the RSSI threshold set. Please see the S2LP_Qi
 * module for details.
 */

#ifndef IEEE802154_S2LP_CSMA_H_
#define IEEE802154_S2LP_CSMA_H_

#include "ieee802154_s2lp_types.h"
#include "ieee802154_s2lp_registers.h"
#include "ieee802154_s2lp_core_spi.h"

/**
 * @brief  Multiplier for Tcca time enumeration (Tcca = Multiplier*Tbit).
 */

typedef enum {
  CSMA_PERIOD_64TBIT,      /*!< CSMA/CA: Sets CCA period to 64*TBIT */
  CSMA_PERIOD_128TBIT,    /*!< CSMA/CA: Sets CCA period to 128*TBIT */
  CSMA_PERIOD_256TBIT,    /*!< CSMA/CA: Sets CCA period to 256*TBIT */
  CSMA_PERIOD_512TBIT    /*!< CSMA/CA: Sets CCA period to 512*TBIT */
}SCsmaPeriod;

/**
  * @brief  S2LP CSMA Init structure definition
  */
typedef struct {
  SFunctionalState  xCsmaPersistentMode;    /*!< Enable the CSMA persistent mode */
  SCsmaPeriod       xMultiplierTbit;        /*!< Set the Tbit multiplier to obtain the Tcca. @ref CcaPeriod */
  uint8_t           xCcaLength;             /*!< Set the Tcca multiplier to determinate the Tlisten. From 0 to 15. */
  uint8_t           cMaxNb;                 /*!< Specifies the max number of backoff cycles. From 0 to 7. */
  uint16_t          nBuCounterSeed;         /*!< Specifies the BU counter seed. */
  uint8_t           cBuPrescaler;           /*!< Specifies the BU prescaler. From 0 to 63. */
} SCsmaInit;

void S2LPCsmaInit(SCsmaInit* pxSCsmaInit);
void S2LPCsmaGetInfo(SCsmaInit* pxSCsmaInit);
void S2LPCsma(SFunctionalState xNewState);
SFunctionalState S2LPCsmaGetCsma(void);
void S2LPCsmaPersistentMode(SFunctionalState xNewState);
SFunctionalState S2LPCsmaGetPersistentMode(void);
void S2LPCsmaSeedReloadMode(SFunctionalState xNewState);
SFunctionalState S2LPCsmaGetSeedReloadMode(void);
void S2LPCsmaSetBuCounterSeed(uint16_t nBuCounterSeed);
uint16_t S2LPCsmaGetBuCounterSeed(void);
void S2LPCsmaSetBuPrescaler(uint8_t cBuPrescaler);
uint8_t S2LPCsmaGetBuPrescaler(void);
void S2LPCsmaSetCcaPeriod(SCsmaPeriod xMultiplierTbit);
uint8_t S2LPCsmaGetCcaPeriod(void);
void S2LPCsmaSetCcaLength(uint8_t xCcaLength);
uint8_t S2LPCsmaGetCcaLength(void);
void S2LPCsmaSetMaxNumberBackoff(uint8_t cMaxNb);
uint8_t S2LPCsmaGetMaxNumberBackoff(void);


#endif