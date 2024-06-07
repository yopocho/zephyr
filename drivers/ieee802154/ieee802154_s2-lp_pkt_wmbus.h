/**
 * @file    S2LP_PktWMbus.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP WMbus packets.
 * @details
 *
 * This module can be used to manage the configuration of S2-LP WMbus
 * packets.
 * The user can obtain a packet configuration filling the structure
 * <i>@ref PktWMbusInit</i>, defining in it some general parameters
 * for the S2-LP WMbus packet format.
 * Since the WMbus protocol is a standard, the configuration of a WMbus
 * packet is very simple to do.
 *
 * <b>Example:</b>
 * @code
 *
 * PktWMbusInit mbusInit={
 *   WMbus_SUBMODE_S1_S2_LONG_HEADER,    // WMbus submode selection
 *   36,                                // added "01" chips on preamble
 *   16                                 // postamble length in "01" chips
 * };
 *
 * ...
 *
 * S2LPPktWMbusInit(&mbusInit);
 *
 * ...
 *
 * @endcode
 *
 * The module provides some other functions that can be used to modify
 * or read only some configuration parameters.
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

#ifndef IEEE802154_S2_LP_PKT_WMMBUS_H_
#define IEEE802154_S2_LP_PKT_WMMBUS_H_

#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_types.h"
#include "ieee802154_s2-lp_packet_handler.h"
#include "ieee802154_s2-lp_core_spi.h"

/**
 * @brief  WMbus submode enumeration.
 */

typedef enum {
  WMBUS_SUBMODE_NOT_CONFIGURED            = 0,   /*!< WMBUS submode S1, S2 (long header) - Header length = WMBUS_prmbl_ctrl + 279 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits) */
  WMBUS_SUBMODE_S1_S2_LONG_HEADER,               /*!< WMBUS submode S1, S2 (long header) - Header length = WMBUS_prmbl_ctrl + 279 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits) */
  WMBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER,       /*!< WMBUS submode S1-m, S2, T2 (other to meter) - Header length = WMBUS_prmbl_ctrl + 15 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits)*/
  WMBUS_SUBMODE_T1_T2_METER_TO_OTHER,            /*!< WMBUS submode T1, T2 (meter to other) - Header length = WMBUS_prmbl_ctrl + 19 (in "01" bit pairs) ,  Sync word = 0x3D (length 10 bits)*/
  WMBUS_SUBMODE_R2_SHORT_HEADER,                 /*!< WMBUS submode R2, short header - Header length = WMBUS_prmbl_ctrl + 39 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits)*/
} WMbusSubmode;

/**
 * @brief  S2LP WMBUS Packet Init structure definition
 */
typedef struct {
  WMbusSubmode   xWMbusSubmode;      /*!< Set the WMBUS submode. @ref WMbusSubmode */
  uint8_t        cPreambleLength;    /*!< Set the preamble length in chip sequence */
  uint8_t        cPostambleLength;   /*!< Set the postamble length in chip sequence */
} PktWMbusInit;

/**
 * @brief  Sets the PREAMBLE field length for S2LP Basic packets.
 * @param  xPreambleLength length of PREAMBLE field in bytes.
 *         This parameter can be any value of @ref BasicPreambleLength.
 * @retval None.
 */
#define S2LPPktWMbusSetPreambleLength(xPreambleLength)                S2LPPktCommonSetPreambleLength((PktPreambleLength)xPreambleLength)

/**
 * @brief  Returns the PREAMBLE field length mode for S2LP Basic packets.
 * @param  None.
 * @retval uint8_t Preamble field length in bytes.
 */
#define S2LPPktWMbusGetPreambleLength()                               S2LPPktCommonGetPreambleLength()

void S2LPPktWMbusInit(PktWMbusInit* pxPktWMbusInit);
void S2LPPktWMbusGetInfo(PktWMbusInit* pxPktWMbusInit);
void S2LPPktWMbusSetFormat(void);
void S2LPPktWMbusSetPostamble(uint8_t cPostamble);
uint8_t S2LPPktWMbusGetPostamble(void);
void S2LPPktWMbusSetPostamblePattern(uint8_t cPostamble);
uint8_t S2LPPktWMbusGetPostamblePattern(void);
void S2LPPktWMbusSetSubmode(WMbusSubmode xWMbusSubmode);
WMbusSubmode S2LPPktWMbusGetSubmode(void);
void S2LPPktWMbusSetPayloadLength(uint16_t nPayloadLength);
uint16_t S2LPPktWMbusGetPayloadLength(void);



#endif
