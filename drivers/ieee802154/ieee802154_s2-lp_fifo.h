/**
 * @file    ieee802154_s2-lp_fifo.h
 * @author  ST Microelectronics
 * @version 1.3.0
 * @date    June, 2019
 * @brief   Configuration and management of S2-LP Fifo.
 * @details
 *
 * This module allows the user to manage the linear FIFO. The functions exported
 * here can be used to set the thresholds for the FIFO almost full / empty alarm
 * interrupts or to get the total number of elements inside the FIFO.
 */

#ifndef IEEE802154_S2_LP_FIFO_H_
#define IEEE802154_S2_LP_FIFO_H_

#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_types.h"
#include "ieee802154_s2-lp_core_spi.h"

uint8_t S2LPFifoReadNumberBytesRxFifo(void);
uint8_t S2LPFifoReadNumberBytesTxFifo(void);
void S2LPFifoSetAlmostFullThresholdRx(uint8_t cThrRxFifo);
uint8_t S2LPFifoGetAlmostFullThresholdRx(void);
void S2LPFifoSetAlmostEmptyThresholdRx(uint8_t cThrRxFifo);
uint8_t S2LPFifoGetAlmostEmptyThresholdRx(void);
void S2LPFifoSetAlmostFullThresholdTx(uint8_t cThrTxFifo);
uint8_t S2LPFifoGetAlmostFullThresholdTx(void);
void S2LPFifoSetAlmostEmptyThresholdTx(uint8_t cThrTxFifo);
uint8_t S2LPFifoGetAlmostEmptyThresholdTx(void);
void S2LPFifoMuxRxFifoIrqEnable(SFunctionalState xNewState);

#endif 