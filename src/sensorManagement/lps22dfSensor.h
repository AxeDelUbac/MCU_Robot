#ifndef LPS22DF_SENSOR_H
#define LPS22DF_SENSOR_H

#include <stdbool.h>

void lps22dfSensor_init(void);
void lps22dfSensor_debug(void);
bool lps22dfSensor_read(float *pressure_hpa, float *temperature_c);

#endif /* LPS22DF_SENSOR_H */
