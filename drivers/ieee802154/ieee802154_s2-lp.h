/**
 * WIP:
 * Relevant pins:
 *  6 SDN Digital in Shutdown input pin. SDN should be = ‘0’ in all modes, except shutdown mode
 *  16 SDO Digital out SPI slave data output
 *  17 SDI Digital in SPI slave data input
 *  18 SCLK Digital in SPI slave clock input
 *  19 CSn Digital in SPI chip select (active low)
 *
 * Guide(s): https://docs.zephyrproject.org/latest/kernel/drivers/index.html
 *           https://www.marcusfolkesson.se/blog/write-a-device-driver-for-zephyr-part3/
*/

/**
 * @file    s2lp.h
 * @author  ST Microelectronics
 * @brief   Wrapper of S2LP Library for BUS IO, include file
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
 * <h2><center>&copy; COPYRIGHT 2021 STMicroelectronics</center></h2>
 */

#ifndef IEEE802154_S2_LP_H_
#define IEEE802154_S2_LP_H_

#include <stdint.h>
#include <stdbool.h>
#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_commands.h"
#include "ieee802154_s2-lp_csma.h"
#include "ieee802154_s2-lp_gpio.h"
#include "ieee802154_s2-lp_timer.h"
#include "ieee802154_s2-lp_fifo.h"
#include "ieee802154_s2-lp_radio.h"
#include "ieee802154_s2-lp_qi.h"
#include "ieee802154_s2-lp_types.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

typedef S2LPStatus StatusBytes;

/*!
 * Generic SX126X error code
 */
#define S2LP_OK      0
#define S2LP_ERROR  -1


#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/

#define S2LP_DIG_DOMAIN_XTAL_THRESH  30000000        /*!< Digital domain logic threshold for XTAL in MHz */

#define XTAL_FREQUENCY          50000000U 

/** XTAL frequency offset compensation value in Hertz
  * Please, take into account that if nominal frequency is 50 MHz and
  * measured XTAL frequency is (for example) 50000157, then XTAL_FREQUENCY_OFFSET must be
  * set to -157, If not avaialble set it to 0 */
#define XTAL_FREQUENCY_OFFSET                   0

#define S2LP_RX_FIFO_SIZE   128
#define S2LP_TX_FIFO_SIZE   128
#define S2LP_CMD_SIZE   2
#define S2LP_BUF_SIZE   S2LP_TX_FIFO_SIZE


#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

/* Exported types ------------------------------------------------------------*/

/*!
 * @brief Initializes the SPI object 
 * @param  none
 * @retval Error code: 0 ok, negative KO
 */
typedef int32_t     (*S2LPBus_Init_Func)(void);   
typedef int32_t     (*S2LPBus_DeInit_Func)(void); 
typedef int32_t     (*S2LPBus_WriteBuf_Func)( uint8_t *, uint8_t *, uint16_t );
typedef void        (*S2LPBus_Delay)(uint32_t );   

typedef struct
{
  S2LPBus_Init_Func            Init;
  S2LPBus_DeInit_Func          DeInit;
  S2LPBus_WriteBuf_Func        WriteBuffer;
  S2LPBus_Delay                Delay;
} S2LP_IO_t;

struct s2lp_config {
  struct spi_dt_spec bus;
  struct gpio_dt_spec sdn;
  struct gpio_dt_spec interrupt;

  //TODO: zephyr CONFIG settings
  /**
   * Find appropriate place for these in some other struct, high importance TODOTODOTODO FIXME
   * VERY possibly that these might just have to be config options in Kconfig.s2-lp!
   * cc1200 does something similar where it uses presets with preset known messages for easy setting registers.
   * If I want fine control of all the radio setttings I'll either need functions for each setting to convert and
   * set that specific setting (although reuse of some register set functions might be possible),
   * or I'm going to need a giant header file of lookup tables that convert setting to register value/cmd
   * which is what cc1200 does.
  */
  // uint32_t lFrequencyBase;
  // uint32_t ModulationSelect;
  // uint32_t lDatarate;
  // uint32_t lFreqDev;
  // uint32_t lBandwidt;
  // config->irq_gpio_selected,
  // config->my_addr,
  // config->multicast_addr,
  // config->broadcast_addr
};

/*Structure to manage External PA */
typedef enum
{
  FEM_SHUTDOWN	= 0x00,
  FEM_TX_BYPASS	= 0x01,
  FEM_TX		= 0x02,
  FEM_RX		= 0x03,
} FEM_OperationType;

/* Exported constants --------------------------------------------------------*/    
                                
/* Exported macros --------------------------------------------------------*/    


/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

 /**
  * @brief  Function
  * @param  pointer to IO functions
  * @retval error status
  */
int32_t S2LP_RegisterBusIO (S2LP_IO_t *pIO);
        
/*!
 * @brief Initialises the bus for S2LP driver communication
 * @param none
 * @retval forward error code from the bsp BUS Init
 */
int32_t S2LP_Init( void );


uint16_t S2LP_WriteRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer );

uint16_t S2LP_ReadRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer );

uint16_t S2LP_SendCommand(uint8_t cCommandCode);

StatusBytes S2LP_WriteFIFO(uint8_t cNbBytes, uint8_t* pcBuffer);

StatusBytes S2LP_ReadFIFO(uint8_t cNbBytes, uint8_t* pcBuffer);

int32_t S2LP_RcoCalibration(void);

#endif