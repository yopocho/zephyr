/**
 * @file    ieee802154_s2-lp_qi.c
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP QI.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */

#include "ieee802154_s2-lp_qi.h"

#define IS_RSSI_MODE(MODE)            (MODE<=3)
#define IS_RSSI_DBM(VAL)              ((VAL>=-146) && (VAL<=-2))
#define IS_RSSI_FLT(VAL)              (VAL<=15)
#define IS_PQI_LVL_CHECK(VAL)         (VAL<=15)
#define IS_SQI_LVL_CHECK(VAL)         (VAL<=7)
#define S2LPRadioRegToRssidBm(VAL)    (VAL - 146)
#define S2LPRadioRssidBmToReg(VAL)    (uint8_t)(VAL+146)

/**
 * @brief  Returns the RSSI value.
 * @param  None.
 * @retval int32_t RSSI value.
 */
int32_t S2LPRadioGetRssidBm(void)
{
  uint8_t tmp;
  *(uint8_t*)&g_xStatus = S2LPSpiReadRegisters(RSSI_LEVEL_ADDR, 1, &tmp);
  return S2LPRadioRegToRssidBm((int32_t)tmp);
}

/**
 * @brief  Returns the RSSI value from a current RX mode.
 * @param  None.
 * @retval uint8_t RSSI value.
 */
int32_t S2LPRadioGetRssidBmRun(void)
{
  uint8_t tmpBuffer[2];

  /*The first value is the previous one, so the first read operation clear this value,
               than it is read the actual during RX. */
  *(uint8_t*)&g_xStatus = S2LPSpiReadRegisters(RSSI_LEVEL_RUN_ADDR, 2, tmpBuffer);

  return S2LPRadioRegToRssidBm((int32_t)tmpBuffer[1]);
}



/**
 * @brief  Set the RSSI threshold in dBm.
 * @param  rssiThrehsold in dBm.
 * @retval None.
 */
void S2LPRadioSetRssiThreshdBm(int32_t wRssiThrehsold)
{
  uint8_t tmp;
  s_assert_param(IS_RSSI_DBM(wRssiThrehsold));

  tmp = S2LPRadioRssidBmToReg(wRssiThrehsold);
  *(uint8_t*)&g_xStatus = S2LPSpiWriteRegisters(RSSI_TH_ADDR, 1, &tmp);
}



/**
* @brief  Initialize the RSSI measurement.
* @param  Pointer to the SRssiInit struct to use fot the initialization.
* @retval None.
*/
void S2LPRadioRssiInit(SRssiInit* xSRssiInit)
{
  uint8_t tmpBuffer[2];

  s_assert_param(IS_RSSI_MODE(xSRssiInit->xRssiMode));
  s_assert_param(IS_RSSI_DBM(xSRssiInit->cRssiThreshdBm));
  s_assert_param(IS_RSSI_FLT(xSRssiInit->cRssiFlt));

  /* Reads the PCKT_FLT_OPTIONS rrgister */
  S2LPSpiReadRegisters(RSSI_FLT_ADDR, 1, &tmpBuffer[0]);

  /* Enables or disables filtering on my address */
  tmpBuffer[0] &= ~(RSSI_FLT_REGMASK | CS_MODE_REGMASK);
  tmpBuffer[0] |= (xSRssiInit->cRssiFlt<<4);
  tmpBuffer[0] |= (xSRssiInit->xRssiMode<<2);

  tmpBuffer[1] = S2LPRadioRssidBmToReg(xSRssiInit->cRssiThreshdBm);

  *(uint8_t*)&g_xStatus = S2LPSpiWriteRegisters(RSSI_FLT_ADDR, 2, tmpBuffer);
}


/**
* @brief  Return the RSSI measurements information to be filled with the information.
* @param  Pointer to the SRssiInit struct.
*               It will be filled with RSSI Mode, RSSI filter gain value and RSSI Threshold.
* @retval None.
*/
void S2LPRadioGetRssiInfo(SRssiInit* xSRssiInit)
{
  uint8_t tmpBuffer[2];

  S2LPSpiReadRegisters(RSSI_FLT_ADDR, 2, tmpBuffer);

  xSRssiInit->xRssiMode = (SRssiMode)((tmpBuffer[0]&CS_MODE_REGMASK)>>2);
  xSRssiInit->cRssiFlt = (tmpBuffer[0]&RSSI_FLT_REGMASK)>>4;
  xSRssiInit->cRssiThreshdBm = S2LPRadioRegToRssidBm(tmpBuffer[1]);

}


/**
* @brief  Enable or disables the CS blanking.
* @param  xCsBlank enable or disable.
* @retval None.
*/
void S2LPRadioCsBlanking(SFunctionalState xCsBlank)
{
  uint8_t tmp;

  s_assert_param(IS_SFUNCTIONAL_STATE(xCsBlank));

  S2LPSpiReadRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);

  if(xCsBlank == S_ENABLE) {
    tmp |= CS_BLANKING_REGMASK;
  } else {
    tmp &= ~CS_BLANKING_REGMASK;
  }
  *(uint8_t*)&g_xStatus = S2LPSpiWriteRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);

}


/**
* @brief  Enable or Disable the antenna switching.
* @param  xAntennaSwitch enable or disable.
* @retval None.
*/
void S2LPRadioAntennaSwitching(SFunctionalState xAntennaSwitch)
{
  uint8_t tmp;

  s_assert_param(IS_SFUNCTIONAL_STATE(xAntennaSwitch));

  S2LPSpiReadRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);

  if(xAntennaSwitch == S_ENABLE) {
    tmp |= AS_ENABLE_REGMASK;
  } else {
    tmp &= ~AS_ENABLE_REGMASK;
  }
  *(uint8_t*)&g_xStatus = S2LPSpiWriteRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);

}


/**
* @brief  Set the PQI threshold.
* @param  PQI_LEVEL.
* @retval None.
*/
void S2LPRadioSetPQIThreshold(uint8_t cPQIThreshold)
{
  uint8_t tmp;
  s_assert_param(IS_PQI_LVL_CHECK(cPQIThreshold));

  S2LPSpiReadRegisters(QI_ADDR, 1, &tmp);
  tmp &= ~PQI_TH_REGMASK;
  tmp |= (((uint8_t)cPQIThreshold)<<1);
  S2LPSpiWriteRegisters(QI_ADDR, 1, &tmp);
}

/**
* @brief  Set the SQI threshold.
* @param  SQI_LEVEL.
* @retval None.
*/
void S2LPRadioSetSQIThreshold(uint8_t cSQIThreshold)
{
  uint8_t tmp;
  s_assert_param(IS_SQI_LVL_CHECK(cSQIThreshold));

  S2LPSpiReadRegisters(QI_ADDR, 1, &tmp);
  tmp &= ~SQI_TH_REGMASK;
  tmp |= (((uint8_t)cSQIThreshold)<<5);
  S2LPSpiWriteRegisters(QI_ADDR, 1, &tmp);
}

/**
* @brief  Set the SQI enable.
* @param  SQI_ENABLE.
* @retval None.
*/
void S2LPRadioEnableSQI(SFunctionalState xSQIEnable)
{
  uint8_t tmp;
  s_assert_param(IS_SFUNCTIONAL_STATE(xSQIEnable));

  S2LPSpiReadRegisters(QI_ADDR, 1, &tmp);
  tmp &= ~SQI_EN_REGMASK;
  tmp |= (((uint8_t)xSQIEnable));
  S2LPSpiWriteRegisters(QI_ADDR, 1, &tmp);
}

/**
* @brief  Return the CS (carrier sense) indication.
* @param  None.
* @retval CS indication.
*/
SFlagStatus S2LPQiGetCs(void)
{
  uint8_t tmp;

  S2LPSpiReadRegisters(LINK_QUALIF1_ADDR, 1, &tmp);
  tmp = (tmp&CS_REGMASK)>>7;
  return (SFlagStatus)tmp;
}