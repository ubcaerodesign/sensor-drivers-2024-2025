/*
 * MS4525DO.h
 *
 *  Created on: Oct 26, 2024
 *      Author: chant
 */

//Datasheet: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS4525DO%7FB10%7Fpdf%7FEnglish%7FENG_DS_MS4525DO_B10.pdf%7FCAT-BLPS0002
/*Differential Pressure Transducer Used for 2024/2025 (PN): MS4525DO-DS3BK001DPL*/

#ifndef INC_MS4525DO_H_
#define INC_MS4525DO_H_

#include "stm32f1xx_hal.h" /* Needed for I2C */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/*MS4525DO CONFIGURATION PARAMETERS*/
#define TYPE_MS4525DO     		(uint8_t)  0 /*1 - Type A, 0 - Type B*/
#define PMAX_PSI_MS4525DO 		(double)   1 //casted to double for transfer function
#define PMIN_PSI_MS4525DO 		(double)   -1
#define ADDRESS_I2C_MS4525DO 	(uint8_t) 0x46
#define AIR_DENSITY			 	(double) 1.225 //kg/m^3

//#define VERBOSE_MODE_EN //uncomment to enable verbose debug mode
//#define WIND_TUNNEL_EN //uncomment to enable wind tunnel calibration features (enabled during wind tunnel only)
#define PRINTF_OVERLOAD //uncomment to have printf print to serial

/*CALIBRATION PARAMETERS*/
//record betz reading in wind tunnel and raw pressure counts (decimal) from 200RPM to 550RPM in 50 RPM increments
#define WINDTUNNEL_BETZ_200RPM (double) 0
#define WINDTUNNEL_BETZ_250RPM (double) 1.15
#define WINDTUNNEL_BETZ_300RPM (double) 4.7
#define WINDTUNNEL_BETZ_350RPM (double) 8.55
#define WINDTUNNEL_BETZ_400RPM (double) 12
#define WINDTUNNEL_BETZ_450RPM (double) 16.25
#define WINDTUNNEL_BETZ_500RPM (double) 20.3
#define WINDTUNNEL_BETZ_550RPM (double) 24.8

#define RAW_PRESSURE_DEC_200RPM (uint16_t) 8223
#define RAW_PRESSURE_DEC_250RPM (uint16_t) 8231
#define RAW_PRESSURE_DEC_300RPM (uint16_t) 8262
#define RAW_PRESSURE_DEC_350RPM (uint16_t) 8294
#define RAW_PRESSURE_DEC_400RPM (uint16_t) 8333
#define RAW_PRESSURE_DEC_450RPM (uint16_t) 8409
#define RAW_PRESSURE_DEC_500RPM (uint16_t) 8419
#define RAW_PRESSURE_DEC_550RPM (uint16_t) 8463

/*linear interpolation macro*/
#define LINEAR_INTERPOLATE(x,x1,x2,y1,y2) (double) (y1+(x-x1)*(y2-y1)/(x2-x1))

/*Raw bytes*/
struct raw_t {
	uint16_t pressure;
	uint16_t temperature;
};
/*Processed Data*/
struct processed_t {
	double pressure_psi; /*range: [-1,1]*/
	double temperature_C; /*range: []*/
	double airspeed_mps; /*range: */
	double airspeed_calibrated_mps;
};
/*data to be sent over via CAN*/
struct CAN_payload_t {
	uint16_t airspeed; 		//conversion?
	uint16_t temperature;	//conversion?
	//more flags to be added
	uint8_t is_stale: 1;	//data freshness
	uint8_t i2c_comms_error: 1;
};
/*I2C read status codes - see MS4525DO interface manual*/
typedef enum {
	normal,
	reserved,
	stale,
	fault,
	unknown
} SensorStatus;

struct MS4525DO_t {
	I2C_HandleTypeDef *i2c_handle;
	SensorStatus sensor_status;
	struct raw_t raw_data;
	struct processed_t processed_data;
	struct CAN_payload_t CAN_package;
};

void MS4525DO_Initialize(struct MS4525DO_t *pSensor, I2C_HandleTypeDef *hi2c);
void read_MS4525DO(struct MS4525DO_t *pSensor);
double calibrate_airspeed(uint16_t raw_pressure, double uncalibrated_airspeed);
double calibrate_airspeed_LUT(uint16_t raw_pressure);
#endif /* INC_MS4525DO_H_ */
