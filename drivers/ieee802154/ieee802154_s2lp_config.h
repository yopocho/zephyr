/**
 * @file    ieee802154_s2-lp_config.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   S2LP Configuration and useful defines .
 * @details
 *
 * This file is used to include all or a part of the S2LP
 * libraries into the application program which will be used.
 * Moreover some important parameters are defined here and the
 * user is allowed to edit them.
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
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 */

#ifndef IEEE802154_S2_LP_CONFIG_H_
#define IEEE802154_S2_LP_CONFIG_H_

#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_commands.h"
#include "ieee802154_s2-lp_csma.h"
#include "ieee802154_s2-lp_general.h"
#include "ieee802154_s2-lp_gpio.h"
#include "ieee802154_s2-lp_timer.h"
#include "ieee802154_s2-lp_fifo.h"
#include "ieee802154_s2-lp_packet_handler.h"
#include "ieee802154_s2-lp_pkt_basics.h"
#include "ieee802154_s2-lp_pkt_wmbus.h"
#include "ieee802154_s2-lp_pkt_stack.h"
#include "ieee802154_s2-lp_radio.h"
#include "ieee802154_s2-lp_qi.h"
#include "ieee802154_s2-lp_types.h"
#include "ieee802154_s2-lp_core_spi.h"

#define DIG_DOMAIN_XTAL_THRESH  30000000        /*!< Digital domain logic threshold for XTAL in MHz */

#endif 