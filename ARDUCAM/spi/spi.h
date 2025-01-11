#ifndef SRC_SPI_SPI_H_
#define SRC_SPI_SPI_H_

#include <stdint.h>
#include "main.h"

void spiBegin(void);
uint8_t spiReadWriteByte(uint8_t TxData);
void spiCsLow(int pin);
void spiCsHigh(int pin);
void spiCsOutputMode(int pin);

#endif /* SRC_SPI_SPI_H_ */
