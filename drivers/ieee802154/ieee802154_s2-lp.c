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
    
    uint32_t ret = spi_write_dt(&config->bus, &tx_data);
    if (ret != 0) {
        return ret;
    }

  return 0;
}

/** Driver Initialization */
int32_t S2LP_Init(const struct device *dev)
{
  // Leaving this codeblock here while figuring out what to do with it

  // if (IO_func.Init()<0)
  // {
  //   return S2LP_ERROR;
  // }
  // return S2LP_OK;

  const struct s2lp_config *config = dev->config;

  /* Configure GPIOs */
	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s is not ready",
			config->interrupt.port->name);
		return -ENODEV;
	}
	gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);

  if(!gpio_is_ready_dt(&config->sdn)) {
    LOG_ERR("GPIO port %s is not ready",
      config->sdn.port->name);
    return -ENODEV
  }
  gpio_pin_configure_dt(&config->sdn, GPIO_OUTPUT);

  if(!spi_is_ready_dt(&config->bus)) {
    LOG_ERR("SPI bus %s is not ready", config->bus);
    return -ENODEV
  }
  
  S2LPCmdStrobeSres();
  
  /*IRQ setup and init for S2LP GPIOs*/
  SGpioInit xGpioIRQ= {
    (uint8_t)CONFIG_IEEE802154_S2LP_IRQ_GPIO_SELECTED,
    S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
    S2LP_GPIO_DIG_OUT_IRQ
  };

  S2LPGpioInit(&xGpioIRQ);

  //TODO: Add these to some struct in ieee802154_s2-lp.h or somewhere suitable 
  SRadioInit xRadioInit = {
    s2lp_base_config.lFrequencyBase,
    s2lp_base_config.ModulationSelect,
    s2lp_base_config.lDatarate,
    s2lp_base_config.lFreqDev,
    s2lp_base_config.lBandwidt
  };

  S2LPRadioInit(&xRadioInit);

  //TODO: /perhaps/ make this variable, maybe add an enum with some relevant maximums for local jurisdictions
  S2LPRadioSetMaxPALevel(S_DISABLE);
  S2LPRadioSetPALeveldBm(7,12); //12 is randomly selected rn
  S2LPRadioSetPALevelMaxIndex(7);

  //TODO: Add accurate settings for these
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

  //TODO: Add addresses
  PktBasicAddressesInit xAddressInit={
    S_ENABLE,          /* Filtering my address */
    my_address,        /* My address */
    S_ENABLE,          /* Filtering multicast address */
    multicast_address, /* Multicast address */
    S_ENABLE,          /* Filtering broadcast address */
    broadcast_address  /* broadcast address */
  };

  S2LPPktBasicAddressesInit(&xAddressInit);

  //TODO: CSMA settings, could possibly be added to Kconfig
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

  //TODO: Possibly add these to Kconfig
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

//TODO: BIG TODO: implement these mandatory zephyr api features. See ieee802154_b91.c and perhaps ieee802154_cc1200.c :)
// https://docs.zephyrproject.org/apidoc/latest/structieee802154__radio__api.html
static const struct ieee802154_radio_api s2lp_radio_api = {
	.iface_api.init	= s2lp_iface_init,
	.get_capabilities	= s2lp_get_capabilities,
	.cca			= s2lp_cca,
	.set_channel		= s2lp_set_channel,
	.set_txpower		= s2lp_set_txpower,
	.tx			= s2lp_tx,
	.start			= s2lp_start,
	.stop			= s2lp_stop,
	.attr_get		= s2lp_attr_get,
};

static const struct s2lp_config s2lp_cfg = {
    .bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
    .sdn = GPIO_DT_SPEC_INST_GET(0, sdn_gpios),
    .interrupt = GPIO_DT_SPEC_INST_GET(0, int_gpios)
};

//TODO: s2lp_data is currently irrelevant, a solution between the driver and zephyr is probably needed here
DEVICE_DT_INST_DEFINE(0, &S2LP_Init, NULL, &s2lp_data, &s2lp_cfg, 
        POST_KERNEL, CONFIG_IEEE802154_S2LP_INIT_PRIORITY, 
        &s2lp_radio_api);