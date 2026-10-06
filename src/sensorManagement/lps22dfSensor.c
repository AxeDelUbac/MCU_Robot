#include "lps22dfSensor.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

static const struct device *const lps22df_dev =
    DEVICE_DT_GET(DT_NODELABEL(iks4a1_lps22df));

static bool lps22df_ready;

void lps22dfSensor_init(void)
{
    lps22df_ready = device_is_ready(lps22df_dev);
    if (!lps22df_ready) {
        printk("LPS22DF: sensor not ready\r\n");
    }
}

void lps22dfSensor_debug(void)
{
    float pressure_hpa;
    float temperature_c;
    if (!lps22dfSensor_read(&pressure_hpa, &temperature_c)) {
        return;
    }

    int pressure_centi_hpa = (int)(pressure_hpa * 100.0f);
    int temperature_centi = (int)(temperature_c * 100.0f);
    printk("LPS22DF: pressure=%d.%02d hPa temperature=%d.%02d C\r\n",
        pressure_centi_hpa / 100, abs(pressure_centi_hpa % 100),
        temperature_centi / 100, abs(temperature_centi % 100));
}

bool lps22dfSensor_read(float *pressure_hpa, float *temperature_c)
{
    if (!lps22df_ready || pressure_hpa == NULL || temperature_c == NULL) {
        return false;
    }

    struct sensor_value pressure;
    struct sensor_value temperature;
    int ret = sensor_sample_fetch(lps22df_dev);
    if (ret == 0) {
        ret = sensor_channel_get(lps22df_dev, SENSOR_CHAN_PRESS, &pressure);
    }
    if (ret == 0) {
        ret = sensor_channel_get(lps22df_dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
    }
    if (ret != 0) {
        printk("LPS22DF: sample read failed (%d)\r\n", ret);
        return false;
    }

    *pressure_hpa = ((float)pressure.val1 + (float)pressure.val2 * 1e-6f) * 10.0f;
    *temperature_c = (float)temperature.val1 + (float)temperature.val2 * 1e-6f;
    return true;
}
