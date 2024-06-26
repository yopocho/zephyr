/**
 * @file    S2LP_Timer_ex.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP Timers using floating point.
 * @details
 *
 * This module provides API to configure the S2-LP timing mechanisms using floating points.
 *
 * <b>Example:</b>
 * @code
 *   ...
 *
 *   S2LPTimerSetRxTimeoutMs(50.0);
 * 
 *   ...
 *
 * @endcode
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
 * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
 */

#ifndef IEEE802154_S2_LP_TIMER_EX_H_
#define IEEE802154_S2_LP_TIMER_EX_H_

#include "ieee802154_s2-lp_core_spi.h"

void S2LPTimerSetRxTimerMs(float fDesiredMsec);
void S2LPTimerGetRxTimer(float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler);
void S2LPTimerSetWakeUpTimerMs(float fDesiredMsec);
void S2LPTimerSetWakeUpTimerReloadMs(float fDesiredMsec);
void S2LPTimerGetWakeUpTimer(float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti);
void S2LPTimerGetWakeUpTimerReload(float* pfWakeUpReloadMsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti);
void S2LPTimerComputeWakeUpValues(float fDesiredMsec , uint8_t* pcCounter , uint8_t* pcPrescaler);





#endif