/**
 * @file    S2LP_CORE_SPI.h
 * @author  ST Microelectronics
 * @brief   Legacy include file to let S2LP Library compile
 * @details
 */

#ifndef S2LP_CORE_SPI_H_
#define S2LP_CORE_SPI_H_

#include "ieee802154_s2lp.h"

#define		S2LPSpiReadRegisters(...)       S2LP_ReadRegister(__VA_ARGS__)
#define		S2LPSpiWriteRegisters(...)      S2LP_WriteRegister(__VA_ARGS__)
#define     S2LPSpiWriteFifo(...)           S2LP_WriteFIFO(__VA_ARGS__)
#define     S2LPSpiReadFifo(...)            S2LP_ReadFIFO(__VA_ARGS__)
#define     S2LPSpiCommandStrobes(...)      S2LP_SendCommand(__VA_ARGS__)

#endif /*__S2LP_CORE_SPI_H__*/
