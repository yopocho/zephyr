/**
 * @file    ieee802154_s2-lp.c
 * @author  ST Microelectronics
 * @brief   Wrapper of S2LP Library for BUS IO
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

#include "ieee802154_s2-lp.h"

LOG_MODULE_REGISTER(ieee802154_s2lp, CONFIG_IEEE802154_DRIVER_LOG_LEVEL);

#define MAX_RCO_ERR 3

/*!
 * @brief IO function pointer structure
 */
static S2LP_IO_t IO_func;

int32_t S2LP_RegisterBusIO(S2LP_IO_t *pIO)
{
  
  IO_func.Init            = pIO->Init;
  IO_func.DeInit          = pIO->DeInit;
  IO_func.WriteBuffer     = pIO->WriteBuffer;
  IO_func.Delay           = pIO->Delay;
  
  if(!IO_func.Init)
  {
    return S2LP_ERROR;
  }
  return S2LP_OK;

/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @param  dev: Devicetree handle for s2-lp radio
* @return a value from spi_write().
*/ 
int s2lp_write_reg(uint8_t *pcHeader, uint8_t *pcBuffer, uint16_t cNbBytes, const struct device *dev) {

  const struct s2lp_config *config = dev->config;
  
  /*Trasmit header*/
  const struct spi_buf_header tx_buf = 
  {
    .buf = pcHeader, 
    .len = S2LP_CMD_SIZE
  };
  struct spi_buf_set_header tx =
  {
    .buffers = tx_buf,
    .count = 1
  }
  if(cNbBytes) {
    /*Transmit data if needed*/
    LOG_DBG("Writing to register(s):\n\rAddress: %x\n\rData: %x\n\r", *pcHeader, *pcBuffer);
    const struct struct spi_buf tx = 
    {
      .buf = pcBuffer, 
      .len = cNbBytes
    };
    struct spi_buf_set tx =
    {
      .buffers = tx,
      .count = 1
    }
    
    ret = spi_write_dt(&config->bus, &tx_data);
    if (ret != 0) {
        return ret;
    }

  return 0;
}

int32_t S2LP_Init(const struct device *dev)
{
  // if (IO_func.Init()<0)
  // {
  //   return S2LP_ERROR;
  // }
  // return S2LP_OK;

  const struct s2lp_config *config = dev->config;

  if(!spi_is_ready_dt(&config->bus)) {
    LOG_ERR("SPI bus %s is not ready", config->bus);
    return -ENODEV
  }
  
  //TODO: Add zeohyr gpio inits
  // struct gpio_dt_spec sdn_spec = GPIO_DT_SPEC_GET_OR();
  
  // gpio_pin_configure_dt();
  
  S2LPCmdStrobeSres();

  //TODO: IRQ is not setup according to zephyr, FIXME
  /*IRQ setup and init for S2LP GPIOs*/
  SGpioInit xGpioIRQ= {
    irq_gpio_selected,
    S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
    S2LP_GPIO_DIG_OUT_IRQ
  };

  S2LPGpioInit(&xGpioIRQ);

  //TODO: change over to some new struct with this data, refer to STM32Duino git
  SRadioInit xRadioInit = {
    config->lFrequencyBase,
    config->ModulationSelect,
    config->lDatarate,
    config->lFreqDev,
    config->lBandwidth
  };

  S2LPRadioInit(&xRadioInit);

  //TODO: /perhaps/ make this variable, maybe add an enum with some relevant maximums for local jurisdictions
  S2LPRadioSetMaxPALevel(S_DISABLE);
  S2LPRadioSetPALeveldBm(7,12); //12 is randomly selected rn
  S2LPRadioSetPALevelMaxIndex(7);

  PktBasicInit xBasicInit={
    16,                 /* Preamble length */
    32,                 /* Sync length */
    0x88888888,         /* Sync word */
    S_ENABLE,           /* Variable length */
    S_DISABLE,          /* Extended length field */
    PKT_CRC_MODE_8BITS, /* CRC mode */
    S_ENABLE,           /* Enable address */
    S_DISABLE,          /* Enable FEC */
    S_ENABLE            /* Enable Whitening */
  };

 S2LPPktBasicInit(&xBasicInit);

  PktBasicAddressesInit xAddressInit={
    S_ENABLE,          /* Filtering my address */
    my_address,        /* My address */
    S_ENABLE,          /* Filtering multicast address */
    multicast_address, /* Multicast address */
    S_ENABLE,          /* Filtering broadcast address */
    broadcast_address  /* broadcast address */
  };

  S2LPPktBasicAddressesInit(&xAddressInit);

  SCsmaInit xCsmaInit={
    S_ENABLE,           /* Persistent mode enable/disable */
    CSMA_PERIOD_64TBIT, /* CS Period */
    3,                  /* CS Timeout */
    5,                  /* Max number of backoffs */
    0xFA21,             /* BU counter seed */
    32                  /* CU prescaler */
  };

  S2LPCsmaInit(&xCsmaInit);
  S2LPPacketHandlerSetRxPersistentMode(S_ENABLE);

  SRssiInit xSRssiInit = {
    .cRssiFlt = 14,
    .xRssiMode = RSSI_STATIC_MODE,
    .cRssiThreshdBm = -60,
  };
  S2LPRadioRssiInit(&xSRssiInit);

  S2LP_RcoCalibration();

  /* Enable PQI */
  S2LPRadioSetPQIThreshold(0x00);
  S2LPRadioSetPQIThreshold(S_ENABLE);

  /* S2LP IRQs enable */
  S2LPGpioIrqDeInit(NULL);
  S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);
  S2LPGpioIrqConfig(TX_DATA_SENT , S_ENABLE);

  /* clear FIFO if needed */
  S2LPCmdStrobeFlushRxFifo();

  /* Set infinite Timeout */
  S2LPTimerSetRxTimerCounter(0);
  S2LPTimerSetRxTimerStopCondition(ANY_ABOVE_THRESHOLD);

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();

  uint8_t tmp = 0x90;
  S2LPSpiWriteRegisters(0x76, 1, &tmp);

  /* Go to RX state */
  S2LPCmdStrobeCommand(CMD_RX);

  //TODO: Implement irq bindings and handler here
  // attachInterrupt(irq_pin, irq_handler, FALLING);

  

  return 0;
}


/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
* @retval Device status
*/ 
uint16_t S2LP_WriteRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer, const struct device *dev)
{
    uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,cRegAddress};
    uint16_t status;
  
    // IO_func.WriteBuffer( header, pcBuffer, cNbBytes );
    s2lp_write_reg(header, pcBuffer, cNbBytes, dev);
    

    ((uint8_t*)&status)[1]=header[0];
    ((uint8_t*)&status)[0]=header[1]; 
  
    return status;
}

/**
* @brief  Read single or multiple registers.
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval Device status
*/
uint16_t S2LP_ReadRegister(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer, const struct device *dev)
{
    uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,cRegAddress};
    uint16_t status;

    // IO_func.WriteBuffer( header, pcBuffer, cNbBytes );
    s2lp_write_reg(header, pcBuffer, cNbBytes, dev);

    ((uint8_t*)&status)[1]=header[0];
    ((uint8_t*)&status)[0]=header[1]; 
  
    return status;
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval Device status
*/
uint16_t S2LP_SendCommand(uint8_t cCommandCode, const struct device *dev)
{
  uint8_t header[S2LP_CMD_SIZE]={COMMAND_HEADER,cCommandCode};
  uint16_t status;

  // IO_func.WriteBuffer( header, NULL, 0 );
  s2lp_write_reg(header, NULL, 0, dev);
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}

/**
* @brief  Write data into TX FIFO.
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval Device status
*/
StatusBytes S2LP_WriteFIFO(uint8_t cNbBytes, uint8_t* pcBuffer, const struct device *dev)
{
  uint8_t header[S2LP_CMD_SIZE]={WRITE_HEADER,LINEAR_FIFO_ADDRESS};
  StatusBytes status;

  // IO_func.WriteBuffer( header, pcBuffer, cNbBytes );
  s2lp_write_reg(header, pcBuffer, cNbBytes, dev);
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}


/**
* @brief  Read data from RX FIFO.
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval Device status
*/
StatusBytes S2LP_ReadFIFO(uint8_t cNbBytes, uint8_t* pcBuffer, const struct device *dev)
{
  uint8_t header[S2LP_CMD_SIZE]={READ_HEADER,LINEAR_FIFO_ADDRESS};
  StatusBytes status;

  // IO_func.WriteBuffer( header, pcBuffer, cNbBytes );
  s2lp_write_reg(header, pcBuffer, cNbBytes, dev);
  
  ((uint8_t*)&status)[1]=header[0];
  ((uint8_t*)&status)[0]=header[1];
  
  return status;
}


/**
  * @brief  FunctionDescription
  * @retval Error code:  S2LP_OK on success, S2LP_ERROR on error during calibration of RCO.
  */
int32_t S2LP_RcoCalibration(void)
{
  uint8_t tmp[2],tmp2;
  int32_t nRet = S2LP_OK;
  uint8_t nErr = 0;

  S2LP_ReadRegister(XO_RCO_CONF0_ADDR, 1, &tmp2);
  tmp2 |= RCO_CALIBRATION_REGMASK;
  S2LP_WriteRegister(XO_RCO_CONF0_ADDR, 1, &tmp2);  /* Enable the RCO CALIB setting bit to 1 */

  S2LP_CMD_StrobeStandby();
  IO_func.Delay(50);
  S2LP_CMD_StrobeReady();

  do
  {
    S2LPSpiReadRegisters(MC_STATE1_ADDR, 1, tmp);
    
    //Check RCO Calibration Error and retry MAX_RCO_ERR times
    if ((tmp[0]&ERROR_LOCK_REGMASK)==1) 
    {
      //Disable TimerCalibrationRco
      S2LPSpiReadRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
      tmp2 &= ~RCO_CALIBRATION_REGMASK;
      S2LPSpiWriteRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
      
      //Enable TimerCalibrationRco
      S2LPSpiReadRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
      tmp2 |= RCO_CALIBRATION_REGMASK;
      S2LPSpiWriteRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
      nErr++;
    }    
  }
  while(((tmp[0]&RCO_CAL_OK_REGMASK)==0) && nErr <= MAX_RCO_ERR);

  if (nErr < MAX_RCO_ERR)
  {
    S2LPSpiReadRegisters(RCO_CALIBR_OUT4_ADDR, 2, tmp);
    S2LPSpiReadRegisters(RCO_CALIBR_CONF2_ADDR, 1, &tmp2);
    
    tmp[1]=(tmp[1]& RFB_IN_0_REGMASK)|(tmp2&(~RFB_IN_0_REGMASK)); 
    
    S2LPSpiWriteRegisters(RCO_CALIBR_CONF3_ADDR, 2, tmp);
    
    S2LPSpiReadRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
    tmp2 &= ~RCO_CALIBRATION_REGMASK;
    S2LPSpiWriteRegisters(XO_RCO_CONF0_ADDR, 1, &tmp2);
  }
  else
  {
    nRet = S2LP_ERROR;
  }

  return nRet;

}

/**
  * @brief  S2LP_TCXOInit
  * @retval None
  */
void S2LP_TCXOInit(void)
{
  uint8_t tmp;
  S2LP_ReadRegister(XO_RCO_CONF0_ADDR, 1, &tmp);
  tmp|=EXT_REF_REGMASK;
  S2LP_WriteRegister(XO_RCO_CONF0_ADDR, 1, &tmp);
}

static const struct s2lp_config s2lp_cfg = {
    .bus = SPI_DT_SPEC_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0)
};

DEVICE_DT_DEFINE(0, &S2LP_Init, NULL, &s2lp_data, &s2lp_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, NULL);