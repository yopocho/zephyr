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

#ifndef IEEE802154_S2LP_H_
#define IEEE802154_S2LP_H_

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/random/random.h>

#include <zephyr/sys/atomic.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "ieee802154_s2lp_registers.h"
#include "ieee802154_s2lp_commands.h"
#include "ieee802154_s2lp_csma.h"
#include "ieee802154_s2lp_gpio.h"
#include "ieee802154_s2lp_timer.h"
#include "ieee802154_s2lp_fifo.h"
#include "ieee802154_s2lp_radio.h"
#include "ieee802154_s2lp_qi.h"
#include "ieee802154_s2lp_types.h"

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


struct s2lp_config_base {
  uint32_t lFrequencyBase;
  uint32_t ModulationSelect;
  uint32_t lDatarate;
  uint32_t lFreqDev;
  uint32_t lBandwidt;
  uint8_t my_addr[8];
  uint8_t multicast_addr[8];
  uint8_t broadcast_addr[8];
};

//TODO: fix my_addr, multicast_addr, broadcast_addr, they're supposed to be somewhere rather else than here
#ifdef CONFIG_IEEE802154_S2LP_DEFAULT_PRESET
struct s2lp_config_base s2lp_base_config {
  .lFrequencyBase = 868300000;
  .ModulationSelect = MOD_2FSK;
  .lDatarate = 125000;
  .lFreqDev = 62500;
  .lBandwidt = 100000;
  uint8_t my_addr[8] = {0x00,0x00,0x00,0x00,CONFIG_IEEE802154_MAC4,CONFIG_IEEE802154_MAC5,CONFIG_IEEE802154_MAC6,CONFIG_IEEE802154_MAC7};
  uint8_t multicast_addr[8];
  uint8_t broadcast_addr[8];
};
#endif

struct s2lp_config {
  struct spi_dt_spec bus;
  struct gpio_dt_spec sdn;
  struct gpio_dt_spec interrupt;
};

//TODO: Runtime device information
// struct s2lp_context {
//   struct net_if *iface;
// 	/**************************/
// 	struct gpio_callback rx_tx_cb;
// 	uint8_t mac_addr[8];
// 	/************RF************/
// 	const struct cc1200_rf_registers_set *rf_settings;
// 	/************TX************/
// 	struct k_sem tx_sync;
// 	atomic_t tx;
// 	atomic_t tx_start;
// 	/************RX************/
// 	K_KERNEL_STACK_MEMBER(rx_stack,
// 			      CONFIG_IEEE802154_CC1200_RX_STACK_SIZE);
// 	struct k_thread rx_thread;
// 	struct k_sem rx_lock;
// 	atomic_t rx;
// }

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

int s2lp_write_reg(uint8_t *pcHeader, uint8_t *pcBuffer, uint16_t cNbBytes);

uint16_t S2LP_WriteRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer );

uint16_t S2LP_ReadRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer );

uint16_t S2LP_SendCommand(uint8_t cCommandCode);

StatusBytes S2LP_WriteFIFO(uint8_t cNbBytes, uint8_t* pcBuffer);

StatusBytes S2LP_ReadFIFO(uint8_t cNbBytes, uint8_t* pcBuffer);

int32_t S2LP_RcoCalibration(void);

#endif