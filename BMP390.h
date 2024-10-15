/*
 * BMP390.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Ethan Gray
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include "stm32f1xx_hal.h" /* Needed for I2C */

/*
 * DEFINES
 */
#define BMP390_I2C_ADDR (0x76 << 1) /* SDO = 0 --> 0x76, SDO = 1 --> 0x77 */

#define BMP390_CHIP_ID 0x60

#define BMP390_LEN_CALIB_DATA 21 /* Number of calibration data registers */

/*
 * REGISTERS
 */
#define BMP390_REG_CHIP_ID 0x00
#define BMP390_REG_REV_ID 0x01
#define BMP390_REG_ERR 0x02
#define BMP390_REG_STATUS 0x03
#define BMP390_REG_PRESS_7_0 0x04
#define BMP390_REG_PRESS_15_8 0x05
#define BMP390_REG_PRESS_23_16 0x06
#define BMP390_REG_TEMP_7_0 0x07
#define BMP390_REG_TEMP_15_8 0x08
#define BMP390_REG_TEMP_23_16 0x09
#define BMP390_REG_TIME_7_0 0x0C
#define BMP390_REG_TIME_15_8 0x0D
#define BMP390_REG_TIME_23_16 0x0E
#define BMP390_REG_EVENT 0x10
#define BMP390_REG_INT_STATUS 0x11
#define BMP390_REG_FIFO_LENGTH_7_0 0x12
#define BMP390_REG_FIFO_LENGTH_8_11 0x13
#define BMP390_REG_FIFO_DATA 0x14
#define BMP390_REG_FIFO_WTM_7_0 0x15
#define BMP390_REG_FIFO_WTM_8 0x16
#define BMP390_REG_FIFO_CONFIG_1 0x17
#define BMP390_REG_FIFO_CONFIG_2 0x18
#define BMP390_REG_INT_CTRL 0x19
#define BMP390_REG_IF_CONF 0x1A
#define BMP390_REG_PWR_CTRL 0x1B
#define BMP390_REG_OSR 0x1C
#define BMP390_REG_ODR 0x1D
#define BMP390_REG_CONFIG 0x1F
#define BMP390_REG_CMD 0x7E
#define BMP390_REG_CALIB_DATA 0x31


/*
 * STRUCTURES
 */
struct bmp3_reg_calib_data
{
    /*! Trim Variables */

    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;
};

struct bmp3_quantized_calib_data
{
    /*! Quantized Trim Variables */

    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
};

struct bmp3_calib_data
{
    /*! Quantized data */
    struct bmp3_quantized_calib_data quantized_calib_data;

    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};

/*
 * SENSOR STRUCT
 */
struct BMP390 {
	I2C_HandleTypeDef *i2cHandle;

	float press_Pa;
	float temp_C;

	struct bmp3_calib_data calib_data;
};

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

/*
 * INITIALIZATION
 */
uint8_t BMP390_Initialize(struct BMP390 *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * READ DATA
 */
HAL_StatusTypeDef BMP390_ReadData(struct BMP390 *dev);

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef BMP390_ReadRegister(struct BMP390 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMP390_ReadRegisters(struct BMP390 *dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef BMP390_WriteRegister(struct BMP390 *dev, uint8_t reg, uint8_t *data);


#endif /* INC_BMP390_H_ */
