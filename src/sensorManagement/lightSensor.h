#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "sensorManagement.h"

void  lightSensor_begin(void);

float lightSensor_readLuminosityValue(void);
float lightSensor_readRawLuminosity(void);
float lightSensor_readFilteredLuminosityValue(void);

void  lightSensor_debug(void);

#endif /* LIGHT_SENSOR_H */
