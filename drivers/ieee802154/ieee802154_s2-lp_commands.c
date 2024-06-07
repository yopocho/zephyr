/**
 * @file    S2LP_Commands.c
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Management of S2-LP Commands.
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

#include "ieee802154_s2-lp_commands.h"

#define IS_S2LP_CMD(CMD)  (CMD == CMD_TX || \
                             CMD == CMD_RX || \
                             CMD == CMD_READY || \
                             CMD == CMD_STANDBY || \
                             CMD == CMD_SLEEP || \
                             CMD == CMD_LOCKRX || \
                             CMD == CMD_LOCKTX || \
                             CMD == CMD_SABORT || \
                             CMD == CMD_LDC_RELOAD || \
                             CMD == CMD_SEQUENCE_UPDATE || \
                             CMD == CMD_SRES || \
                             CMD == CMD_FLUSHRXFIFO || \
                             CMD == CMD_FLUSHTXFIFO \
                            )

                            
/**
 * @brief  Send a specific command to S2LP.
 * @param  xCommandCode code of the command to send.
           This parameter can be any value of @ref S2LPCmd.
 * @retval None.
 */
void S2LPCmdStrobeCommand(S2LPCmd xCommandCode)
{
  /* Check the parameters */
  s_assert_param(IS_S2LP_CMD(xCommandCode));

  *(uint8_t*)&g_xStatus = S2LPSpiCommandStrobes((uint8_t) xCommandCode);
}
