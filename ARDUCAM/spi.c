/*
 * This file is part of the Arducam SPI Camera project.
 *
 * Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 */
#include "spi.h"

extern SPI_HandleTypeDef hspi3;

/*uint8_t arducamSpiTransfer(uint8_t TxData)
{
    uint8_t RxData = 0;
    if (HAL_SPI_TransmitReceive(&hspi3, &TxData, &RxData, 1, 10) == HAL_OK)
    {
        return RxData;
    }
    return 0;  // Timeout/Error
}*/
uint8_t arducamSpiTransfer(uint8_t TxData)
{
    uint8_t RxData = 0;
    HAL_StatusTypeDef status;

    // Perform SPI Transfer
    status = HAL_SPI_TransmitReceive(&hspi3, &TxData, &RxData, 1, 100);

    // Check the result of the SPI transaction
    if (status == HAL_OK) {
        return RxData;  // Successful transfer
    }

    // Error handling based on the status code
    switch (status) {
        case HAL_ERROR:
            printf("SPI Error: HAL_ERROR occurred during SPI transfer.\n");
            break;
        case HAL_BUSY:
            printf("SPI Error: HAL_BUSY - SPI is busy.\n");
            break;
        case HAL_TIMEOUT:
            printf("SPI Error: HAL_TIMEOUT occurred during SPI transfer.\n");
            break;
        default:
            printf("SPI Error: Unknown error code %d.\n", status);
            break;
    }

    // Return 0 to indicate error or timeout
    return 0;
}

void Camera_WriteRegister(uint8_t regAddr, uint8_t value) {
    uint8_t data[2] = {0x80 | regAddr, value};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, data, 2, HAL_MAX_DELAY); // Send register and data
	if (status != HAL_OK) {
		// Error during transmit
		printf("SPI transmit error\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS High
		return;
	}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // CS High
}

uint8_t Camera_ReadRegister(uint8_t regAddr) {
    uint8_t data[2] = {regAddr & 0x7F, 0x00}; // Register address + dummy byte
    uint8_t received[2];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_SPI_TransmitReceive(&hspi3, data, received, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // CS High

    return received[1]; // Return the second byte (register value)
}

void arducamSpiCsPinLow(int pin)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_RESET);
}

void arducamSpiCsPinHigh(int pin)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_SET);
}
