#ifndef SRC_SPI_SPI_H_
#define SRC_SPI_SPI_H_

#include <stdint.h>
#include "main.h"
#include <stdio.h>

#define OV5642_TEST_REG 0x00
#define WRITE_CMD 0x80           // OV5642 SPI write command
#define READ_CMD  0x00           // OV5642 SPI read command

//void arducamSpiCsPinLow(int pin);
//void arducamSpiCsPinHigh(int pin);
//void Camera_WriteRegister(uint8_t regAddr, uint8_t value);
//void Camera_WriteRegister16(uint16_t regAddr, uint8_t value);
//uint8_t Camera_ReadRegister(uint8_t regAddr);
//uint8_t Camera_ReadRegister16(uint16_t regAddr);

#endif /* SRC_SPI_SPI_H_ */
