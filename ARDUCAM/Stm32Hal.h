#ifndef SRC_STM32HAL_H_
#define SRC_STM32HAL_H_

#include "delay/delay.h"
#include "spi/spi.h"

#define arducamSpiBegin()        spiBegin()
#define arducamSpiTransfer(val)  spiReadWriteByte(val) //  SPI communication sends a byte
#define arducamSpiCsPinHigh(pin) spiCsHigh(pin)        // Set the CS pin of SPI to high level
#define arducamSpiCsPinLow(pin)  spiCsLow(pin)         // Set the CS pin of SPI to low level
#define arducamCsOutputMode(pin) spiCsOutputMode(pin)
#define arducamDelayMs(val)      delayMs(val) //  Delay Ms
#define arducamDelayUs(val)      delayUs(val) // Delay Us

#endif /* SRC_STM32HAL_H_ */
