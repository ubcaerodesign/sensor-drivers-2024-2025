#include "spi.h"
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

// Function to initialize SPI interface
void spiBegin(void)
{
	MX_SPI1_Init();
}

// Function to send and receive a byte via SPI
uint8_t spiReadWriteByte(uint8_t TxData)
{
    uint8_t RxData = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }
    return RxData;
}

// Function to configure the SPI chip select (CS) pin as output
void spiCsOutputMode(int pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t CSPin = CAMERA_CS_PIN;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = CSPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Function to pull the CS pin low (activate the chip)
void spiCsLow(int pin)
{
    uint32_t CSPin = CAMERA_CS_PIN;
    HAL_GPIO_WritePin(GPIOA, CSPin, GPIO_PIN_RESET);
}

// Function to pull the CS pin high (deactivate the chip)
void spiCsHigh(int pin)
{
    uint32_t CSPin = CAMERA_CS_PIN;
    HAL_GPIO_WritePin(GPIOA, CSPin, GPIO_PIN_SET);
}
