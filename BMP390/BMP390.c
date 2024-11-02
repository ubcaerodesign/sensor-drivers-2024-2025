/*
 * BMP390.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Ethan Gray
 */

#include "BMP390.h"

/*
 * Calibration functions
 */
static HAL_StatusTypeDef get_calib_data(struct BMP390 *dev);
static void parse_calib_data(const uint8_t *reg_data, struct BMP390 *dev);
static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct bmp3_quantized_calib_data *calib_data);
static float BMP390_compensate_pressure(uint32_t uncomp_press, struct bmp3_quantized_calib_data *calib_data);

static HAL_StatusTypeDef get_calib_data(struct BMP390 *dev)
{
	HAL_StatusTypeDef rslt;
    uint8_t reg_addr = BMP390_REG_CALIB_DATA;

    /* Array to store calibration data */
    uint8_t calib_data[BMP390_LEN_CALIB_DATA] = { 0 };

    /* Read the calibration data from the sensor */
    rslt = BMP390_ReadRegisters(dev, reg_addr, calib_data, BMP390_LEN_CALIB_DATA);

    /* Parse calibration data and store it in device structure */
    parse_calib_data(calib_data, dev);

    return rslt;
}

static void parse_calib_data(const uint8_t *reg_data, struct BMP390 *dev)
{
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data *reg_calib_data = &dev->calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data *quantized_calib_data = &dev->calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((double)reg_calib_data->par_t1 / temp_var);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((double)reg_calib_data->par_t2 / temp_var);
    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((double)reg_calib_data->par_t3 / temp_var);
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((double)(reg_calib_data->par_p1 - (16384)) / temp_var);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((double)(reg_calib_data->par_p2 - (16384)) / temp_var);
    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((double)reg_calib_data->par_p3 / temp_var);
    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((double)reg_calib_data->par_p4 / temp_var);
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((double)reg_calib_data->par_p5 / temp_var);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((double)reg_calib_data->par_p6 / temp_var);
    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((double)reg_calib_data->par_p7 / temp_var);
    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((double)reg_calib_data->par_p8 / temp_var);
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((double)reg_calib_data->par_p9 / temp_var);
    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((double)reg_calib_data->par_p10 / temp_var);
    reg_calib_data->par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((double)reg_calib_data->par_p11 / temp_var);
}

static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct bmp3_quantized_calib_data *calib_data)
{
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
	/* Returns compensated temperature */
	return calib_data->t_lin;
}

static float BMP390_compensate_pressure(uint32_t uncomp_press, struct bmp3_quantized_calib_data *calib_data)
{
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press *
	(calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

uint8_t BMP390_Initialize(struct BMP390 *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set structure parameters */
	dev->i2cHandle = i2cHandle;
	dev->press_Pa = 0.0f;
	dev->temp_C = 0.0f;

	/* Store number of transaction errors */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check device IDs */
	uint8_t regData;

	status = BMP390_ReadRegister(dev, BMP390_REG_CHIP_ID, &regData);
	errNum += (status != HAL_OK);

	if (regData != BMP390_CHIP_ID) {

		return 255;

	}

	/* Set serial interface */
	regData = 0x02;
	status = BMP390_WriteRegister(dev, BMP390_REG_IF_CONF, &regData);
	errNum += (status != HAL_OK);

	/* Enable sensors in normal mode */
	regData = 0x33;
	status = BMP390_WriteRegister(dev, BMP390_REG_PWR_CTRL, &regData);
	errNum += (status != HAL_OK);

	/* Set oversampling (8x for pressure 1x for temperature) */
	regData = 0x03;
	status = BMP390_WriteRegister(dev, BMP390_REG_OSR, &regData);
	errNum += (status != HAL_OK);

	/* Set subdivision/subsampling (50Hz sampling rate) */
	regData = 0x02;
	status = BMP390_WriteRegister(dev, BMP390_REG_ODR, &regData);
	errNum += (status != HAL_OK);

	/* Set IRR filter coefficient to 3 */
	regData = 0x02;
	status = BMP390_WriteRegister(dev, BMP390_REG_CONFIG, &regData);
	errNum += (status != HAL_OK);

	/* Get calibration data */
	status = get_calib_data(dev);
	errNum += (status != HAL_OK);

	return errNum;

}

/*
 * READ DATA
 */
HAL_StatusTypeDef BMP390_ReadData(struct BMP390 *dev) {

	HAL_StatusTypeDef status;
	uint8_t regData[3];

	/* Read raw values from temperature registers (24 bits) */
	status = BMP390_ReadRegisters(dev, BMP390_REG_TEMP_7_0, regData, 3);

	if (status != HAL_OK) {
		return status;
	}

	/* Combine register values to get raw temperature */
	uint32_t tempRaw = ((regData[2] << 16) | (regData[1] << 8) | regData[0]);

	/* Compensate and store temperature */
	dev->temp_C = BMP390_compensate_temperature(tempRaw, &dev->calib_data.quantized_calib_data);


	/* Read raw values from pressure registers (24 bits)*/
	status = BMP390_ReadRegisters(dev, BMP390_REG_PRESS_7_0, regData, 3);

	if (status != HAL_OK) {
		return status;
	}

	/* Combine register values to get raw pressure */
	uint32_t pressRaw = ((regData[2] << 16) | (regData[1] << 8) | regData[0]);

	/* Compensate and store pressure */
	dev->press_Pa = BMP390_compensate_pressure(pressRaw, &dev->calib_data.quantized_calib_data);

	return status;
}


/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef BMP390_ReadRegister(struct BMP390 *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, BMP390_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMP390_ReadRegisters(struct BMP390 *dev, uint8_t reg, uint8_t *data, uint8_t length) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, BMP390_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}

HAL_StatusTypeDef BMP390_WriteRegister(struct BMP390 *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, BMP390_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}


