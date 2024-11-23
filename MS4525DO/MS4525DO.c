/*
 * MS4525DO.c
 *
 *  Created on: Oct 26, 2024
 *      Author: chantle
 */
#include "MS4525DO.h"

#ifdef PRINTF_OVERLOAD
// Use the handle for the UART you configured (e.g., huart1)
extern UART_HandleTypeDef huart1;
int _write(int file, char *data, int len) {
    // Transmit data via UART
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
#endif
/**
 * Configures which i2c port MS4525DO is on
 */
void MS4525DO_Initialize(struct MS4525DO_t *pSensor, I2C_HandleTypeDef *hi2c) {
	/*Set i2c handle*/
	pSensor->i2c_handle = hi2c;
	/*Initialize everything to defaults*/
	SensorStatus initStatus = normal;
	pSensor->sensor_status = initStatus;
	pSensor->raw_data.pressure = 0;
	pSensor->raw_data.temperature = 0;
	pSensor->processed_data.pressure_psi = 0;
	pSensor->processed_data.temperature_C = 0;
	pSensor->processed_data.airspeed_mps = 0;
	pSensor->processed_data.airspeed_calibrated_mps = 0;
	pSensor->CAN_package.airspeed = 0;
	pSensor->CAN_package.temperature = 0;
	pSensor->CAN_package.is_stale = 0;
	pSensor->CAN_package.i2c_comms_error = 0;
}
void read_MS4525DO(struct MS4525DO_t *pSensor) {
	uint8_t data_buffer[4]; //data buffer to store raw I2C data
	HAL_StatusTypeDef i2c_status = HAL_I2C_Master_Receive(pSensor->i2c_handle, ADDRESS_I2C_MS4525DO << 1, data_buffer, sizeof(data_buffer), HAL_MAX_DELAY);
#ifdef VERBOSE_MODE_EN
    if (i2c_status == HAL_OK) {
        printf("HAL_OK\r\n");
    } else if (i2c_status == HAL_ERROR) {
    	printf("HAL_ERROR\r\n");
    } else if (i2c_status == HAL_BUSY) {
    	printf("HAL_BUSY\r\n");
    } else if (i2c_status == HAL_TIMEOUT) {
        printf("HAL_TIMEOUT\r\n");
    }
    //diagnose HAL error
    uint32_t i2c_error = HAL_I2C_GetError(pSensor->i2c_handle);
    if (i2c_error == HAL_I2C_ERROR_NONE) {
      printf("no errors \r\n");
    } else if (i2c_error == HAL_I2C_ERROR_BERR) {
      printf("HAL_I2C_ERROR_BERR\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_ARLO) {
      printf("HAL_I2C_ERROR_ARLO\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_AF) {
      printf("HAL_I2C_ERROR_AF\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_OVR) {
      printf("HAL_I2C_ERROR_OVR\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_DMA) {
      printf("HAL_I2C_ERROR_DMA\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_TIMEOUT) {
      printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    }
    //print the raw bytes
    printf("%u, ",data_buffer[0]);
    printf("%u, ",data_buffer[1]);
    printf("%u, ",data_buffer[2]);
    printf("%u \r\n",data_buffer[3]);
#endif
    /*Status - 2 Bits*/
    uint8_t read_status = (uint8_t)(data_buffer[0] >> 6);
    SensorStatus stat;
    switch (read_status){
    case 0:
    	stat = normal;
    	break;
    case 1:
    	stat = reserved;
    	break;
    case 2:
    	stat = stale;
    	break;
    case 3:
    	stat = fault;
    	break;
    default:
    	stat = unknown;
    	break;
    }
    pSensor->sensor_status = stat;
    /*Pressure - 14 Bits*/
    pSensor->raw_data.pressure = (((uint16_t)data_buffer[0] << 8) & 0x3F00) + ((uint16_t)data_buffer[1] << 0); 	//Combines High and Low Pressure. Clears status bits
    /*Temperature - 11 Bits*/
    pSensor->raw_data.temperature = ((uint16_t)data_buffer[2] << 3) + ((uint16_t)data_buffer[3] >> 5);			//Combines High and Low Temperature. Clears last 5 bits.

    if(TYPE_MS4525DO) {
    	 /*Type A*/
    	 pSensor->processed_data.pressure_psi = ((((double)pSensor->raw_data.pressure-1638.3)*(PMAX_PSI_MS4525DO - PMIN_PSI_MS4525DO ))/13106.4)+PMIN_PSI_MS4525DO;
    } else {
    	/*Type B*/
    	pSensor->processed_data.pressure_psi = ((((double)pSensor->raw_data.pressure-819.15)*(PMAX_PSI_MS4525DO-PMIN_PSI_MS4525DO))/14744.7)+PMIN_PSI_MS4525DO;
    }
    pSensor->processed_data.temperature_C = (((double)pSensor->raw_data.temperature*200.0)/2047.0)-50.0;

    /*Output swings positive when Port 1> Port 2, negative vice versa. Output is 50% (8192D) when Port 1 = Port 2*/
    if(pSensor->processed_data.pressure_psi >= 0) {
    	//Positive to denote Port 1 > Port 2
    	pSensor->processed_data.airspeed_mps = sqrt((2*6894.7*pSensor->processed_data.pressure_psi)/AIR_DENSITY);
    } else {
        //Negative to denote Port 1 < Port 2
    	pSensor->processed_data.airspeed_mps = -sqrt((2*6894.7*abs(pSensor->processed_data.pressure_psi))/AIR_DENSITY);
    }

    pSensor->processed_data.airspeed_calibrated_mps = calibrate_airspeed(pSensor->raw_data.pressure, pSensor->processed_data.airspeed_mps);

#ifdef WIND_TUNNEL_EN
    printf("%u", pSensor->raw_data.pressure);
    printf(", %f", pSensor->processed_data.pressure_psi);
    printf(", %f", pSensor->processed_data.airspeed_mps);
    printf(", %f \r\n", pSensor->processed_data.airspeed_calibrated_mps);
#endif

    /*Populate CAN package*/
    uint16_t airspeed_tx = (uint8_t)(pSensor->processed_data.airspeed_calibrated_mps*10); 	//multiply by 10 to preserve 1 decimal place
    uint16_t temperature_tx = (uint8_t)(pSensor->processed_data.temperature_C*10);		 	//multiply by 10 to preserve 1 decimal place
    pSensor->CAN_package.airspeed = airspeed_tx;
    pSensor->CAN_package.temperature = temperature_tx;
    if(pSensor->sensor_status == stale) {
    	pSensor->CAN_package.is_stale = 1;
    } else {
    	pSensor->CAN_package.is_stale = 0;
    }
    if((pSensor->sensor_status == reserved) || (pSensor->sensor_status == fault) || (pSensor->sensor_status == unknown)) {
    	pSensor->CAN_package.i2c_comms_error = 1;
    } else {
    	pSensor->CAN_package.i2c_comms_error = 0;
    }
    printf("Airspeed: %u \r\n", pSensor->CAN_package.airspeed);
    printf("Temp: %u \r\n", pSensor->CAN_package.temperature);
    printf("Is Stale: %u \r\n", pSensor->CAN_package.is_stale);
    printf("Comms Err: %u \r\n", pSensor->CAN_package.i2c_comms_error);
}

double calibrate_airspeed(uint16_t raw_pressure, double uncalibrated_airspeed) {
	double calibrated_airspeed = 0;
	if(raw_pressure > RAW_PRESSURE_DEC_550RPM) {
		calibrated_airspeed = uncalibrated_airspeed; //calibration not needed for higher airspeeds
	} else {
		/*calibration needed at lower airspeeds (below 550 RPM in wind tunnel)*/
		/*between calibration points*/
		if((raw_pressure > RAW_PRESSURE_DEC_200RPM) && (raw_pressure < RAW_PRESSURE_DEC_250RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_200RPM, RAW_PRESSURE_DEC_250RPM, WINDTUNNEL_BETZ_200RPM, WINDTUNNEL_BETZ_250RPM);
		} else if((raw_pressure > RAW_PRESSURE_DEC_250RPM) && (raw_pressure < RAW_PRESSURE_DEC_300RPM)){
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_250RPM, RAW_PRESSURE_DEC_300RPM, WINDTUNNEL_BETZ_250RPM, WINDTUNNEL_BETZ_300RPM);
		} else if ((raw_pressure > RAW_PRESSURE_DEC_300RPM) && (raw_pressure < RAW_PRESSURE_DEC_350RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_300RPM, RAW_PRESSURE_DEC_350RPM, WINDTUNNEL_BETZ_300RPM, WINDTUNNEL_BETZ_350RPM);
		} else if((raw_pressure > RAW_PRESSURE_DEC_350RPM) && (raw_pressure < RAW_PRESSURE_DEC_400RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_350RPM, RAW_PRESSURE_DEC_400RPM, WINDTUNNEL_BETZ_350RPM, WINDTUNNEL_BETZ_400RPM);
		} else if ((raw_pressure > RAW_PRESSURE_DEC_400RPM) && (raw_pressure < RAW_PRESSURE_DEC_450RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_400RPM, RAW_PRESSURE_DEC_450RPM, WINDTUNNEL_BETZ_400RPM, WINDTUNNEL_BETZ_450RPM);
		} else if((raw_pressure > RAW_PRESSURE_DEC_450RPM) && (raw_pressure < RAW_PRESSURE_DEC_500RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_450RPM, RAW_PRESSURE_DEC_500RPM, WINDTUNNEL_BETZ_450RPM, WINDTUNNEL_BETZ_500RPM);
		} else if ((raw_pressure > RAW_PRESSURE_DEC_500RPM) && (raw_pressure < RAW_PRESSURE_DEC_550RPM)) {
			calibrated_airspeed = LINEAR_INTERPOLATE(raw_pressure, RAW_PRESSURE_DEC_500RPM, RAW_PRESSURE_DEC_550RPM, WINDTUNNEL_BETZ_500RPM, WINDTUNNEL_BETZ_550RPM);
		} else {
			/*look up table if raw pressure falls exactly on calibration point*/
			calibrated_airspeed = calibrate_airspeed_LUT(raw_pressure);
		}
	}
	return calibrated_airspeed;
}
double calibrate_airspeed_LUT(uint16_t raw_pressure) {
	double mapped_airspeed = 0;
	switch (raw_pressure) {
	case RAW_PRESSURE_DEC_200RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_200RPM;
		break;
	case RAW_PRESSURE_DEC_250RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_250RPM;
		break;
	case RAW_PRESSURE_DEC_300RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_300RPM;
		break;
	case RAW_PRESSURE_DEC_350RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_350RPM;
		break;
	case RAW_PRESSURE_DEC_400RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_400RPM;
		break;
	case RAW_PRESSURE_DEC_450RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_450RPM;
		break;
	case RAW_PRESSURE_DEC_500RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_500RPM;
		break;
	case RAW_PRESSURE_DEC_550RPM:
		mapped_airspeed = WINDTUNNEL_BETZ_550RPM;
		break;
	default:
		break;
	}
	return mapped_airspeed;
}
