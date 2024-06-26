/**
 * @file    S2LP_Types.c
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   This file provides functions to manage S2-LP debug.
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
 *
 */

#include "ieee802154_s2-lp_types.h"
#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_core_spi.h"

/**
 * @brief  S2LP Status global variable.
 *         This global variable of @ref S2LPStatus type is updated on every SPI transaction
 *         to maintain memory of S2LP Status.
 */
volatile S2LPStatus g_xStatus;

#ifdef  S2LP_USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file  pointer to the source file name
 * @param line  assert_param error line source number
 * @retval : None
 */
void s_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
 * @brief  Updates the gState (the global variable used to maintain memory of S2LP Status)
 *         reading the MC_STATE register of S2LP.
 * @param  None
 * @retval None
 */
void S2LPRefreshStatus(void)
{  
  uint8_t tempRegValue;
  /* Read the MC_STATE both from register and from SPI header and exit when they match.
      This will protect against possible transition state changes */
  do
  {
    /* Reads the MC_STATE register to update the g_xStatus */
    *(uint8_t*)&g_xStatus = S2LPSpiReadRegisters(MC_STATE0_ADDR, 1, &tempRegValue);
  }
  while((tempRegValue>>1)!=g_xStatus.MC_STATE);
}
