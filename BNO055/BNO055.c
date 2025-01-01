#include "BNO055.h"

/**
 * GLOBAL
 */
FATFS fs;
FIL fil;

I2C_HandleTypeDef *BNO055_i2c_port;

double theta_F_old = 0, phi_F_old = 0;
double theta = 0, phi = 0;


/**
 * @brief Configures which i2c port bno055 is on
 */
void BNO055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
	BNO055_i2c_port = hi2c_device;
}

/**
 * @brief Write to sensor
 * note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
void BNO055_writeRegister(uint8_t reg, uint8_t data) {
	uint8_t txdata[2] = {reg, data};
	uint8_t status = HAL_I2C_Master_Transmit(BNO055_i2c_port, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), HAL_MAX_DELAY);

	// Debugging
    if (status == HAL_OK) {
      return;
    }

    if (status == HAL_ERROR) {
      printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    } else if (status == HAL_TIMEOUT) {
      printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    } else if (status == HAL_BUSY) {
      printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
    } else {
      printf("Unknown status data %d", status);
    }

    uint32_t error = HAL_I2C_GetError(BNO055_i2c_port);
    if (error == HAL_I2C_ERROR_NONE) {
      return;
    } else if (error == HAL_I2C_ERROR_BERR) {
      printf("HAL_I2C_ERROR_BERR\r\n");
    } else if (error == HAL_I2C_ERROR_ARLO) {
      printf("HAL_I2C_ERROR_ARLO\r\n");
    } else if (error == HAL_I2C_ERROR_AF) {
      printf("HAL_I2C_ERROR_AF\r\n");
    } else if (error == HAL_I2C_ERROR_OVR) {
      printf("HAL_I2C_ERROR_OVR\r\n");
    } else if (error == HAL_I2C_ERROR_DMA) {
      printf("HAL_I2C_ERROR_DMA\r\n");
    } else if (error == HAL_I2C_ERROR_TIMEOUT) {
      printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    }
}

/**
 * @brief Write to sensor, 16 bit address
 * note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
void EEPROM_writeRegister16(uint16_t reg, uint8_t data) {
	//uint8_t txdata[3] = {(reg >> 8) & 0xFF, reg & 0xFF, data};
	//uint8_t status = HAL_I2C_Master_Transmit(BNO055_i2c_port, EEPROM_I2C_ADDR << 1, txdata, sizeof(txdata), HAL_MAX_DELAY);

	uint8_t status = HAL_I2C_Mem_Write(BNO055_i2c_port, EEPROM_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, &data, 1, HAL_MAX_DELAY);

	// Debugging
    if (status == HAL_OK) {
      return;
    }

    if (status == HAL_ERROR) {
      printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
    } else if (status == HAL_TIMEOUT) {
      printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
    } else if (status == HAL_BUSY) {
      printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
    } else {
      printf("Unknown status data %d", status);
    }

    uint32_t error = HAL_I2C_GetError(BNO055_i2c_port);
    if (error == HAL_I2C_ERROR_NONE) {
      return;
    } else if (error == HAL_I2C_ERROR_BERR) {
      printf("HAL_I2C_ERROR_BERR\r\n");
    } else if (error == HAL_I2C_ERROR_ARLO) {
      printf("HAL_I2C_ERROR_ARLO\r\n");
    } else if (error == HAL_I2C_ERROR_AF) {
      printf("HAL_I2C_ERROR_AF\r\n");
    } else if (error == HAL_I2C_ERROR_OVR) {
      printf("HAL_I2C_ERROR_OVR\r\n");
    } else if (error == HAL_I2C_ERROR_DMA) {
      printf("HAL_I2C_ERROR_DMA\r\n");
    } else if (error == HAL_I2C_ERROR_TIMEOUT) {
      printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    }
}


/**
 * @brief Read from sensor
 *  note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
void BNO055_readRegister(uint8_t reg, uint8_t *data, uint8_t len) {

	HAL_I2C_Mem_Read(BNO055_i2c_port, BNO055_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);

}

/**
 * @brief Read from sensor, 16 bit address
 *  note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
void EEPROM_readRegister16(uint16_t reg, uint8_t *data, uint8_t len) {

	HAL_I2C_Mem_Read(BNO055_i2c_port, EEPROM_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, data, len, HAL_MAX_DELAY);

}

void BNO055_setPage(uint8_t page) { BNO055_writeRegister(BNO055_PAGE_ID, page); }

void BNO055_reset() {
	BNO055_writeRegister(BNO055_SYS_TRIGGER, 0x20);
	HAL_Delay(700);
}

void BNO055_setExternalCrystalUse(bool state) {
	BNO055_setPage(0);
	uint8_t tmp = 0;
	BNO055_readRegister(BNO055_SYS_TRIGGER, &tmp, 1);
	tmp |= (state == true) ? 0x80 : 0x0;
	BNO055_writeRegister(BNO055_SYS_TRIGGER, tmp);
}

BNO055_opmode_t BNO055_getOperationMode() {
	BNO055_opmode_t mode;
	BNO055_readRegister(BNO055_OPR_MODE, &mode, 1);
	return mode;
}

void BNO055_setOperationMode(BNO055_opmode_t mode) {
	BNO055_writeRegister(BNO055_OPR_MODE, mode);
	if (mode == OPERATION_MODE_CONFIG) {
	HAL_Delay(19);
	} else {
	HAL_Delay(7);
	}
}

void BNO055_setOperationModeConfig() {
	BNO055_setOperationMode(OPERATION_MODE_CONFIG);
}

void BNO055_setOperationModeNDOF() {
	BNO055_setOperationMode(OPERATION_MODE_NDOF);
}

void BNO055_setAxisRemap(BNO055_axis_remap_config_t remapcode) {
	BNO055_setOperationModeConfig();
	BNO055_writeRegister(BNO055_AXIS_MAP_CONFIG, remapcode);
	BNO055_setOperationModeNDOF();
}

void BNO055_setAxisSign(BNO055_axis_remap_sign_t remapsign) {
	BNO055_setOperationModeConfig();
	BNO055_writeRegister(BNO055_AXIS_MAP_SIGN, remapsign);
	BNO055_setOperationModeNDOF();
}


void BNO055_setup() {
	BNO055_reset();

	uint8_t id = 0;
	BNO055_readRegister(BNO055_CHIP_ID, &id, 1);
	if (id != BNO055_ID) {
	printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
	}
	BNO055_setOperationModeConfig();

	BNO055_writeRegister(BNO055_PWR_MODE, POWER_MODE_NORMAL);
	BNO055_setPage(0);
	BNO055_writeRegister(BNO055_SYS_TRIGGER, 0x0);

	//BNO055_setOperationModeConfig();
	BNO055_setOperationModeNDOF();

}

bool BNO055_getCalibrationState(BNO055_calibration_state_t *calState) {
	BNO055_setPage(0);
	uint8_t cal = 0;

	BNO055_readRegister(BNO055_CALIB_STAT, &cal, 1);

	calState->sys = (cal >> 6) & 0x03;
	calState->gyro = (cal >> 4) & 0x03;
	calState->accel = (cal >> 2) & 0x03;
	calState->mag = cal & 0x03;

	if((calState->sys == 3) && (calState->gyro == 3) && (calState->accel == 3) && (calState->mag == 3)){
	  return true;
	}
	else{
	  return false;
	}

}

void BNO055_getCalibrationData(BNO055_offsets_t *calData, BNO055_calibration_state_t *calState, int flag) {
	uint8_t buffer[22];

	if(BNO055_getCalibrationState(calState)){ //double check that bno is fully calibrated

		BNO055_setOperationModeConfig(); // wait why do i set this in op mode first i thought op mode was for WRITING to registers

		BNO055_readRegister(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

		calData->accel_offset_x = ((buffer[1] << 8) | buffer[0]);
		calData->accel_offset_y = ((buffer[3] << 8) | buffer[2]);
		calData->accel_offset_z = ((buffer[5] << 8) | buffer[4]);

		calData->mag_offset_x = ((buffer[7] << 8) | buffer[6]);
		calData->mag_offset_y = ((buffer[9] << 8) | buffer[8]);
		calData->mag_offset_z = ((buffer[11] << 8) | buffer[10]);

		calData->gyro_offset_x = ((buffer[13] << 8) | buffer[12]);
		calData->gyro_offset_y = ((buffer[15] << 8) | buffer[14]);
		calData->gyro_offset_z = ((buffer[17] << 8) | buffer[16]);

		calData->accel_radius = ((buffer[19] << 8) | buffer[18]);

		calData->mag_radius = ((buffer[21] << 8) | buffer[20]);

		BNO055_setOperationModeNDOF();

	}

	if(flag == 0){
		BNO055_saveCalibrationDataSD(calData); // save new offsets into SD card
	}
	else BNO055_saveCalibrationDataEEPROM(calData); // save new offsets into EEPROM

}

void BNO055_setCalibrationData(BNO055_offsets_t *calData) {
	uint8_t buffer[22]; // 22 bytes

	memcpy(buffer, calData, 22); //STM32 is little-endian so this is fine
	//uint8_t *byteData = (uint8_t *)calData; //try this some time

	BNO055_setOperationModeConfig();

	for (uint8_t i=0; i < 22; i++) {
	    BNO055_writeRegister(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
	}

	BNO055_setOperationModeNDOF();
}

/**
 * Storing offset data starting at address 0x0000 of EEPROM
 */
void BNO055_saveCalibrationDataEEPROM(BNO055_offsets_t *calData){

	uint8_t buffer[22]; // 22 bytes
	memcpy(buffer, calData, sizeof(BNO055_offsets_t));

    for (uint8_t i = 0; i < 22; i++) {
    	EEPROM_writeRegister16(0x0000 + i, buffer[i]);
    	HAL_Delay(10);
    }
    HAL_Delay(1000);

}

/**
 * Read offsets starting at address 0x0000 of EEPROM
 */
void BNO055_loadCalibrationDataEEPROM(){

	BNO055_offsets_t calDataStruct;
	BNO055_offsets_t *calData = &calDataStruct;
    uint8_t buffer[22];

    EEPROM_readRegister16(0x0000, buffer, sizeof(buffer));

	calData->accel_offset_x = (int16_t)((buffer[1] << 8) | buffer[0]);
	calData->accel_offset_y = (int16_t)((buffer[3] << 8) | buffer[2]);
	calData->accel_offset_z = (int16_t)((buffer[5] << 8) | buffer[4]);

	calData->mag_offset_x = (int16_t)((buffer[7] << 8) | buffer[6]);
	calData->mag_offset_y = (int16_t)((buffer[9] << 8) | buffer[8]);
	calData->mag_offset_z = (int16_t)((buffer[11] << 8) | buffer[10]);

	calData->gyro_offset_x = (int16_t)((buffer[13] << 8) | buffer[12]);
	calData->gyro_offset_y = (int16_t)((buffer[15] << 8) | buffer[14]);
	calData->gyro_offset_z = (int16_t)((buffer[17] << 8) | buffer[16]);

	calData->accel_radius = (int16_t)((buffer[19] << 8) | buffer[18]);

	calData->mag_radius = (int16_t)((buffer[21] << 8) | buffer[20]);

	BNO055_displayCalibrationData(calData);
	BNO055_setCalibrationData(calData);

}

void BNO055_saveCalibrationDataSD(BNO055_offsets_t *calData){

	char buffer[77];
	//Each integer might take up to 6 characters for a large value (e.g., 1000000), plus a comma for separation
	//11 integers can require up to 11 * 7 = 77 characters (including commas)

	snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
	calData->accel_offset_x,calData->accel_offset_y, calData->accel_offset_z,
	calData->gyro_offset_x,calData->gyro_offset_y, calData->gyro_offset_z,
	calData->mag_offset_x,calData->mag_offset_y, calData->mag_offset_z,
	calData->accel_radius,calData->mag_radius);

	f_mount(&fs, "", 0);
	f_open(&fil, "BNO055_OFFSETS.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	f_lseek(&fil, 0);  // Start of file
	f_puts(buffer, &fil);
	f_close(&fil);

}

/**
 * THIS IS FOR IF YOU HAVE CALBIRATED ONCE AND SAVED IT TO THE SD CARD
 * Reads the offsets from the .csv file saved on the SD card and applies the offsets to the bno
 */

void BNO055_loadCalibrationDataSD(){

	BNO055_offsets_t calDataStruct;
	BNO055_offsets_t *calData = &calDataStruct;
	char buffer[77];

	f_open(&fil, "BNO055_OFFSETS.csv", FA_READ);

	f_gets(buffer, sizeof(buffer), &fil);

	f_close(&fil);

	sscanf(buffer, "%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd",
			&calData->accel_offset_x,&calData->accel_offset_y, &calData->accel_offset_z,
			&calData->gyro_offset_x,&calData->gyro_offset_y, &calData->gyro_offset_z,
			&calData->mag_offset_x,&calData->mag_offset_y, &calData->mag_offset_z,
			&calData->accel_radius,&calData->mag_radius);


	BNO055_displayCalibrationData(calData);
	BNO055_setCalibrationData(calData);

}

void BNO055_loadCalibrationData(int flag){
	if(flag == 0){
			BNO055_loadCalibrationDataSD(); // save new offsets into SD card
	}
	else BNO055_loadCalibrationDataEEPROM(); // save new offsets into EEPROM
}

void BNO055_displayCalibrationData(BNO055_offsets_t *calData){
    printf("Accel: ");
    printf("%d, %d, %d \r\n", calData->accel_offset_x,calData->accel_offset_y, calData->accel_offset_z);

    printf("Gyro: ");
    printf("%d, %d, %d \r\n", calData->gyro_offset_x,calData->gyro_offset_y, calData->gyro_offset_z);

    printf("Mag: ");
    printf("%d, %d, %d \r\n", calData->mag_offset_x,calData->mag_offset_y, calData->mag_offset_z);

    printf("Accel Radius: ");
    printf("%d \r\n", calData->accel_radius);

    printf("Mag Radius: ");
    printf("%d \r\n", calData->mag_radius);

    HAL_Delay(500);

}

void BNO055_calibrationRoutine(int flag){
	BNO055_calibration_state_t calState;
	BNO055_offsets_t calData;

	BNO055_setPage(0);

	while(! BNO055_getCalibrationState(&calState)){
		printf("sys = %d, gyro = %d, mag = %d, accel = %d \r\n", calState.sys, calState.gyro, calState.mag, calState.accel);
		HAL_Delay(500);

	}
	printf("--------CALIBRATION COMPLETE STORING INTO MEMORY--------\r\n");
	BNO055_getCalibrationData(&calData, &calState, flag);
	BNO055_displayCalibrationData(&calData);
	BNO055_setCalibrationData(&calData);
	printf("--------DONE--------\r\n");

}

void BNO055_getVector(uint8_t vec, BNO055_vector_t *xyz) {
	BNO055_setPage(0);
	uint8_t buffer[8];    // Quaternion needs 8 bytes

	if (vec == BNO055_VECTOR_QUATERNION)
		BNO055_readRegister(vec, buffer, 8);
	else
		BNO055_readRegister(vec, buffer, 6);

	double scale = 1;

	if (vec == BNO055_VECTOR_MAGNETOMETER) {
		scale = magScale;
	} else if (vec == BNO055_VECTOR_ACCELEROMETER ||
		   vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
		scale = accelScale;
	} else if (vec == BNO055_VECTOR_GYROSCOPE) {
		scale = angularRateScale;
	} else if (vec == BNO055_VECTOR_EULER) {
		scale = eulerScale;
	} else if (vec == BNO055_VECTOR_QUATERNION) {
		scale = quaScale;
	}

	if (vec == BNO055_VECTOR_QUATERNION) {
		xyz->w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
		xyz->x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz->y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
		xyz->z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
	} else {
		xyz->x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
		xyz->y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz->z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
	}
}

void BNO055_getVectorAccelerometer(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_ACCELEROMETER, xyz);
}
void BNO055_getVectorMagnetometer(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_MAGNETOMETER, xyz);
}
void BNO055_getVectorGyroscope(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_GYROSCOPE, xyz);
}
void BNO055_getVectorEuler(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_EULER, xyz);
}
void BNO055_getVectorLinearAccel(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_LINEARACCEL, xyz);
}
void BNO055_getVectorGravity(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_GRAVITY, xyz);
}
void BNO055_getVectorQuaternion(BNO055_vector_t *xyz) {
	BNO055_getVector(BNO055_VECTOR_QUATERNION, xyz);
}

void BNO055_getVectorEuler_fromVectorQuaternion(BNO055_vector_t *quaternion,BNO055_vector_t *euler){
	BNO055_vector_t convertedQuaternion;
	BNO055_vector_t changeQuaternion = {.w = 0.5, .x = 0.5, .y = 0.5, .z = 0.5};

	BNO055_multiplyQuaternion(&changeQuaternion, quaternion, &convertedQuaternion);

    double sq_w = convertedQuaternion.w * convertedQuaternion.w;
    double sq_x = convertedQuaternion.x * convertedQuaternion.x;
    double sq_y = convertedQuaternion.y * convertedQuaternion.y;
    double sq_z = convertedQuaternion.z * convertedQuaternion.z;

    double unit = sq_w + sq_x + sq_y + sq_z;
    double test = convertedQuaternion.x * convertedQuaternion.y + convertedQuaternion.z * convertedQuaternion.w;

    /** x = roll, y = pitch, z = yaw **/
    euler->x = atan2(2 * (convertedQuaternion.w * convertedQuaternion.x + convertedQuaternion.y * convertedQuaternion.z), sq_y + sq_w - sq_x - sq_z);
    euler->y = atan2(2 * (convertedQuaternion.w * convertedQuaternion.y - convertedQuaternion.x * convertedQuaternion.z), sq_w + sq_x - sq_y - sq_z);

    // initial conditionals are for catching singularities when converting to Euler
    if (test > 0.4999 * unit) {
    	euler->z = M_PI / 2;
    }
    else if (test < -0.4999 * unit) {
    	euler->z = -M_PI / 2;
    }
    else {
    	euler->z = asin(2 * test / unit);
    }

}

void BNO055_getMagneticHeading
(BNO055_vector_t *refQuaternion, BNO055_vector_t *refGravity, BNO055_vector_t *quaternion, double *heading){

	double theta_M, phi_M;
	double theta_F_new, phi_F_new;
	double Xm, Ym;

	double rot_matrix[3][3] = {
	        {0.0, 0.0, 0.0},
	        {0.0, 0.0, 0.0},
	        {0.0, 0.0, 0.0}};

	BNO055_vector_t diffQuaternion, newGravity, magnetometer;

	BNO055_multiplyQuaternion(refQuaternion, quaternion, &diffQuaternion);
	BNO055_quaternionToMatrix(&diffQuaternion, rot_matrix);
	BNO055_inverseMatrix(rot_matrix);
	BNO055_matrixVectorMultiply(rot_matrix, refGravity, &newGravity);

	BNO055_getVectorMagnetometer(&magnetometer);

	theta_M = atan2(newGravity.x / GRAVITY, newGravity.z / GRAVITY);
	phi_M = atan2(newGravity.y / GRAVITY, newGravity.z / GRAVITY);
	phi_F_new = 0.95 * phi_F_old + 0.05 * phi_M;
	theta_F_new = 0.95 * theta_F_old + 0.05 * theta_M;

	theta = theta * 0.95 + theta_M * 0.05;
	phi = phi * 0.95 + phi_M * 0.05;

	Xm = magnetometer.x * cos(theta) - magnetometer.y * sin(phi) * sin(theta) + magnetometer.z * cos(phi) * sin(theta);
	Ym = magnetometer.y * cos(phi) + magnetometer.z * sin(phi);

	*heading = atan2(Ym, Xm) * 180 / M_PI;

	theta_F_old = theta_F_new;
	phi_F_old = phi_F_new;

	if (*heading < 0) {
		*heading += 360;
	}

}

void BNO055_getReferences(BNO055_vector_t *refQuaternion, BNO055_vector_t *refGravity){
	BNO055_vector_t initialAccel, newAccel;

	BNO055_getVectorAccelerometer(&initialAccel);

	unsigned long start_test = HAL_GetTick();

	// checks if IMU is stationary within defined test duration
	while ( HAL_GetTick() - start_test <= ACCEL_SYNC_TIME ) {
		BNO055_getVectorAccelerometer(&newAccel);

		// checks if acceleration components exceed defined threshold
		if ( !( abs(newAccel.x - initialAccel.x) < ACCEL_SYNC_THRESHOLD &&
				abs(newAccel.y - initialAccel.y) < ACCEL_SYNC_THRESHOLD &&
				abs(newAccel.z - initialAccel.z) < ACCEL_SYNC_THRESHOLD )) {

			// update start time and initial acceleration
			BNO055_getVectorAccelerometer(&initialAccel);
			start_test = HAL_GetTick();
		}
	}

	// get reference gravity vector and quaternion
	BNO055_getVectorAccelerometer(refGravity);
	BNO055_getVectorQuaternion(refQuaternion);
	BNO055_inverseQuaternion(refQuaternion);

}

void BNO055_inverseQuaternion(BNO055_vector_t *q) { //is this right lol
	q->w = q->w;
	q->x = -(q->x);
	q->y = -(q->y);
	q->z = -(q->z);
}

void BNO055_multiplyQuaternion(BNO055_vector_t *q1, BNO055_vector_t *q2, BNO055_vector_t *q3) {
	 q3->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	 q3->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	 q3->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	 q3->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void BNO055_inverseMatrix(double matrix[3][3]){
	double temp;

	// Swap elements across the diagonal
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 3; j++) {
			// Swap matrix[i][j] and matrix[j][i]
			temp = matrix[i][j];
			matrix[i][j] = matrix[j][i];
			matrix[j][i] = temp;
		}
	}
}

void BNO055_quaternionToMatrix(BNO055_vector_t *q, double matrix[3][3]) {
    matrix[0][0] = 2 * (q->w * q->w + q->x * q->x) - 1;
    matrix[0][1] = 2 * (q->x * q->y - q->w * q->z);
    matrix[0][2] = 2 * (q->x * q->z + q->w * q->y);
    matrix[1][0] = 2 * (q->x * q->y + q->w * q->z);
    matrix[1][1] = 2 * (q->w * q->w + q->y * q->y) - 1;
    matrix[1][2] = 2 * (q->y * q->z - q->w * q->x);
    matrix[2][0] = 2 * (q->x * q->z - q->w * q->y);
    matrix[2][1] = 2 * (q->y * q->z + q->w * q->x);
    matrix[2][2] = 2 * (q->w * q->w + q->z * q->z) - 1;
}

void BNO055_matrixVectorMultiply(double matrix[3][3], BNO055_vector_t *refGravity, BNO055_vector_t *newGravity) {
	newGravity->x = matrix[0][0] * refGravity->x + matrix[0][1] * refGravity->y + matrix[0][2] * refGravity->z;
	newGravity->y = matrix[1][0] * refGravity->x + matrix[1][1] * refGravity->y + matrix[1][2] * refGravity->z;
	newGravity->z = matrix[2][0] * refGravity->x + matrix[2][1] * refGravity->y + matrix[2][2] * refGravity->z;

}


