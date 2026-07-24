#ifndef ENVIRONMENTAL_SENSOR_H
#define ENVIRONMENTAL_SENSOR_H

#include "sensorManagement.h"

void  environmentalSensor_begin(void);

float environmentalSensor_readTemperatureValue(void);
float environmentalSensor_readHumidityValue(void);
float environmentalSensor_readPressureValue(void);
float environmentalSensor_readGasValue(void);

float environmentalSensor_readRawTemperature(void);
float environmentalSensor_readRawHumidity(void);
float environmentalSensor_readRawPressure(void);
float environmentalSensor_readRawGas(void);

float environmentalSensor_readFilteredTemperatureValue(void);
float environmentalSensor_readFilteredHumidityValue(void);
float environmentalSensor_readFilteredPressureValue(void);
float environmentalSensor_readFilteredGasValue(void);

void  environmentalSensor_debug(void);

#endif /* ENVIRONMENTAL_SENSOR_H */
