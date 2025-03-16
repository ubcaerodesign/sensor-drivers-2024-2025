#include "spi_comm.h"
#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi3;

// Function to pull the CS pin low (activate the chip)
/*void arducamSpiCsPinLow(int pin)
{
    HAL_GPIO_WritePin(GPIOA, CAMERA_CS_PIN, GPIO_PIN_RESET);
}

// Function to pull the CS pin high (deactivate the chip)
void arducamSpiCsPinHigh(int pin)
{
    HAL_GPIO_WritePin(GPIOA, CAMERA_CS_PIN, GPIO_PIN_SET);
}*/


/*void Camera_WriteRegister(uint8_t regAddr, uint8_t value) {
    uint8_t data[2] = {WRITE_CMD | regAddr, value};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, data, 2, HAL_MAX_DELAY); // Send register and data
	if (status != HAL_OK) {
		// Error during transmit
		printf("SPI transmit error\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS High
		return 0;  // Return 0 on error
	}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // CS High
}

void Camera_WriteRegister16(uint16_t regAddr, uint8_t value) {
    uint8_t data[3] = {
        WRITE_CMD | ((regAddr >> 8) & 0xFF),  // High byte of register address
        WRITE_CMD | (regAddr & 0xFF),         // Low byte of register address
        value                                 // Value to write
    };

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // CS Low
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);       // Send address and value
	if (status != HAL_OK) {
		// Error during transmit
		printf("SPI transmit error\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS High
		return 0;  // Return 0 on error
	}
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);    // CS High
}

uint8_t Camera_ReadRegister(uint8_t regAddr) {
    uint8_t data[2] = {regAddr | READ_CMD, 0x00}; // Register address + dummy byte
    uint8_t received[2];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // CS Low
    HAL_SPI_TransmitReceive(&hspi3, data, received, 2, HAL_MAX_DELAY); // Transmit and receive
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // CS High

    return received[1]; // Return the second byte (register value)
}

uint8_t Camera_ReadRegister16(uint16_t regAddr) {
    uint8_t data[2] = {
        READ_CMD | ((regAddr >> 8) & 0xFF),  // High byte of register address
        READ_CMD | (regAddr & 0xFF)          // Low byte of register address
    };
    uint8_t received[1];  // Buffer for received data

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

	// SPI Transmit: Send the 16-bit register address
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, data, 2, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		// Error during transmit
		printf("SPI transmit error\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS High
		return 0;  // Return 0 on error
	}

	// SPI Receive: Read the 1 byte of data from the register
	status = HAL_SPI_Receive(&hspi3, received, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		// Error during receive
		printf("SPI receive error\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS High
		return 0;  // Return 0 on error
	}

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    printf("Read from register 0x%04X: 0x%02X\n", regAddr, received[0]);

    return received[0];
}*/

