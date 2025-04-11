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
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;

static volatile bool spi_dma_tx_complete = true;
static volatile bool spi_dma_rx_complete = true;
static volatile uint32_t dma_timeout = 1000; // Default timeout in ms

void SPI_DMA_Init(void)
{
    /* Make sure DMA streams are disabled */
    HAL_DMA_DeInit(&hdma_spi3_tx);
    HAL_DMA_DeInit(&hdma_spi3_rx);

    /* Initialize flags */
    spi_dma_tx_complete = true;
    spi_dma_rx_complete = true;
}

/**
 * @brief  Callback for TX DMA completion
 * @param  hspi Pointer to SPI handle
 * @retval None
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        spi_dma_tx_complete = true;
    }
}

/**
 * @brief  Callback for RX DMA completion
 * @param  hspi Pointer to SPI handle
 * @retval None
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        spi_dma_rx_complete = true;
    }
}

/**
 * @brief  Callback for TX/RX DMA completion
 * @param  hspi Pointer to SPI handle
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        spi_dma_tx_complete = true;
        spi_dma_rx_complete = true;
    }
}

/**
 * @brief  Callback for SPI error
 * @param  hspi Pointer to SPI handle
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        printf("SPI DMA error occurred: %ld\n", hspi->ErrorCode);
        spi_dma_tx_complete = true;
        spi_dma_rx_complete = true;
    }
}

/**
 * @brief  Checks if DMA transfer is in progress
 * @retval true if busy, false if idle
 */
bool SPI_DMA_IsBusy(void)
{
    return (!spi_dma_tx_complete || !spi_dma_rx_complete);
}

/**
 * @brief  Wait for DMA transfer completion
 * @retval None
 */
void SPI_DMA_WaitForCompletion(void)
{
    uint32_t timeout = HAL_GetTick() + dma_timeout;

    while (SPI_DMA_IsBusy()) {
        if (HAL_GetTick() >= timeout) {
            printf("SPI DMA timeout occurred\n");
            // Force completion to avoid deadlock
            spi_dma_tx_complete = true;
            spi_dma_rx_complete = true;
            break;
        }
    }
}


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

/**
 * @brief  Write data using DMA
 * @param  txData Data buffer to transmit
 * @param  size Number of bytes to transmit
 * @retval HAL status
 */
HAL_StatusTypeDef Camera_WriteBurstDMA(uint8_t* txData, uint16_t size)
{
    // Make sure any pending DMA transfer is complete
    SPI_DMA_WaitForCompletion();

    spi_dma_tx_complete = false;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi3, txData, size);

    if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS High
        spi_dma_tx_complete = true;
        printf("DMA Transmit error: %d\n", status);
    }

    return status;
}

/**
 * @brief  Read data using DMA
 * @param  rxData Buffer to receive data
 * @param  size Number of bytes to read
 * @retval HAL status
 */
HAL_StatusTypeDef Camera_ReadBurstDMA(uint8_t* rxData, uint16_t size)
{
    uint8_t* dummy_tx = NULL;
    HAL_StatusTypeDef status;

    // Make sure any pending DMA transfer is complete
    SPI_DMA_WaitForCompletion();

    // Allocate dummy TX buffer filled with 0
    dummy_tx = (uint8_t*)malloc(size);
    if (dummy_tx == NULL) {
        printf("Failed to allocate memory for DMA read\n");
        return HAL_ERROR;
    }

    memset(dummy_tx, 0, size);

    spi_dma_rx_complete = false;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    status = HAL_SPI_TransmitReceive_DMA(&hspi3, dummy_tx, rxData, size);

    if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS High
        spi_dma_rx_complete = true;
        printf("DMA Receive error: %d\n", status);
    }

    free(dummy_tx);
    return status;
}

/**
 * @brief  Transmit and receive data using DMA
 * @param  txData Data buffer to transmit
 * @param  rxData Buffer to receive data
 * @param  size Number of bytes to transfer
 * @retval HAL status
 */
HAL_StatusTypeDef Camera_TransmitReceiveDMA(uint8_t* txData, uint8_t* rxData, uint16_t size)
{
    // Make sure any pending DMA transfer is complete
    SPI_DMA_WaitForCompletion();

    spi_dma_tx_complete = false;
    spi_dma_rx_complete = false;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi3, txData, rxData, size);

    if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // CS High
        spi_dma_tx_complete = true;
        spi_dma_rx_complete = true;
        printf("DMA TransmitReceive error: %d\n", status);
    }

    return status;
}

void arducamSpiCsPinLow(int pin)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_RESET);
}

void arducamSpiCsPinHigh(int pin)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_SET);
}
