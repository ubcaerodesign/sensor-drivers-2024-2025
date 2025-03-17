/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#ifndef __SPI_H
#define __SPI_H
#include "stm32h7xx_hal.h"
#include <stdio.h>

uint8_t arducamSpiTransfer(uint8_t TxData);
void Camera_WriteRegister(uint8_t regAddr, uint8_t value);
uint8_t Camera_ReadRegister(uint8_t regAddr);

void arducamSpiCsPinLow(int pin);
void arducamSpiCsPinHigh(int pin);
#endif
