#ifndef BNO055_H
#define BNO055_H

#include "fatfs.h"
#include <string.h>
#include <stdio.h>

typedef struct{

	double filter_lat; // (rad)
    double filter_long; // (rad)
    double filter_alt; // (m)
    double filter_vN; // (m/s)
    double filter_vE; // (m/s)
    double filter_vD; // (m/s)
    double filter_q0;
    double filter_q1;
    double filter_q2;
    double filter_q3;

    float airspeed; //(m/s)
    double time; // (ms)

} sensors_t;

//void SD_saveImage(uint8_t *imageBuff, size_t buffSize);
//void SD_saveSensors(sensors_t *sensor_data);

#endif
