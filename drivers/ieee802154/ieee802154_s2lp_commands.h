#ifndef IEEE802154_S2-LP_COMMANDS_H_
#define IEEE802154_S2-LP_COMMANDS_H_

#include "ieee802154_s2-lp_registers.h"
#include "ieee802154_s2-lp_types.h"
#include "ieee802154_s2-lp_core_spi.h"

/**
  * @brief  S2LP Commands codes enumeration
  */
typedef enum
{
  CMD_TX =  ((uint8_t)(0x60)),                    /*!< Start to transmit; valid only from READY */
  CMD_RX =  ((uint8_t)(0x61)),                    /*!< Start to receive; valid only from READY */
  CMD_READY =  ((uint8_t)(0x62)),                 /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
  CMD_STANDBY =  ((uint8_t)(0x63)),               /*!< Go to STANDBY; valid only from READY */
  CMD_SLEEP = ((uint8_t)(0x64)),                  /*!< Go to SLEEP; valid only from READY */
  CMD_LOCKRX = ((uint8_t)(0x65)),                 /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
  CMD_LOCKTX = ((uint8_t)(0x66)),                 /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
  CMD_SABORT = ((uint8_t)(0x67)),                 /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
  CMD_LDC_RELOAD = ((uint8_t)(0x68)),             /*!< LDC Mode: Reload the LDC timer with the value stored in the  LDC_PRESCALER / COUNTER  registers; valid from all states  */
  CMD_RCO_CALIB =  ((uint8_t)(0x69)),             /*!< Start (or re-start) the RCO calibration */
  CMD_SRES = ((uint8_t)(0x70)),                   /*!< Reset of all digital part, except SPI registers */
  CMD_FLUSHRXFIFO = ((uint8_t)(0x71)),            /*!< Clean the RX FIFO; valid from all states */
  CMD_FLUSHTXFIFO = ((uint8_t)(0x72)),            /*!< Clean the TX FIFO; valid from all states */
  CMD_SEQUENCE_UPDATE =  ((uint8_t)(0x73)),       /*!< Autoretransmission: Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register valid from all states */
} S2LPCmd;

/**
 * @brief  Sends the TX command to S2-LP. Start to transmit.
 * @param  None.
 * @retval None.
 * @note: this macro sets the SMPS switching frequency to 5.46MHz about for ETSI regulation compliancy.
 */
#define S2LPCmdStrobeTx()         {uint8_t tmp=0x9C; S2LPSpiWriteRegisters(0x76,1,&tmp);\
                                        FEM_Operation(FEM_TX); S2LPCmdStrobeCommand(CMD_TX);}


/**
 * @brief  Sends the RX command to S2-LP. Start to receive.
 * @param  None.
 * @retval None.
 * @note: this macro sets the SMPS switching frequency to 3.12MHz.
 */
#define S2LPCmdStrobeRx()         {uint8_t tmp=0x90; S2LPSpiWriteRegisters(0x76,1,&tmp);\
                                    FEM_Operation(FEM_RX); S2LPCmdStrobeCommand(CMD_RX);}

#define S2LPCmdStrobeReady()      S2LPCmdStrobeCommand(CMD_READY)
#define S2LPCmdStrobeStandby()    S2LPCmdStrobeCommand(CMD_STANDBY)
#define S2LPCmdStrobeSleep()      S2LPCmdStrobeCommand(CMD_SLEEP)
#define S2LPCmdStrobeLockRx()     S2LPCmdStrobeCommand(CMD_LOCKRX)
#define S2LPCmdStrobeLockTx()     S2LPCmdStrobeCommand(CMD_LOCKTX)
#define S2LPCmdStrobeSabort()     S2LPCmdStrobeCommand(CMD_SABORT)
#define S2LPCmdStrobeLdcReload()  S2LPCmdStrobeCommand(CMD_LDC_RELOAD)
#define S2LPCmdStrobeSequenceUpdate() S2LPCmdStrobeCommand(CMD_SEQUENCE_UPDATE)
#define S2LPCmdStrobeSres()          S2LPCmdStrobeCommand(CMD_SRES)
#define S2LPCmdStrobeFlushRxFifo()    S2LPCmdStrobeCommand(CMD_FLUSHRXFIFO)
#define S2LPCmdStrobeFlushTxFifo()    S2LPCmdStrobeCommand(CMD_FLUSHTXFIFO)

void S2LPCmdStrobeCommand(S2LPCmd xCommandCode);

#endif