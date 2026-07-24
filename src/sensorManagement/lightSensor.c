#include "lightSensor.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

static const struct device *bh1750_dev;

static float fLuminosityBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fLuminosityFilteredBuffer[MOVING_AVERAGE_BUFFER_SIZE];

static float fLuminosityTreshold[2] = {0.0f, 2500.0f};

void lightSensor_begin(void)
{
    bh1750_dev = device_get_binding("BH1750");
    if (!bh1750_dev || !device_is_ready(bh1750_dev)) {
        LOG_ERR("BH1750 not found or not ready");
        return;
    }
    LOG_INF("BH1750 ready");
}

float lightSensor_readRawLuminosity(void)
{
    if (!bh1750_dev) return 0.0f;
    struct sensor_value val;
    if (sensor_sample_fetch(bh1750_dev) != 0) return 0.0f;
    if (sensor_channel_get(bh1750_dev, SENSOR_CHAN_LIGHT, &val) != 0) return 0.0f;
    return (float)val.val1 + (float)val.val2 * 1e-6f;
}

float lightSensor_readLuminosityValue(void)
{
    float raw = lightSensor_readRawLuminosity();
    float avg = sensorManagement_movingAverage(fLuminosityBuffer, raw);
    sensorManagement_isTresholdReached(LUMINOSITY, avg, fLuminosityTreshold);
    return avg;
}

float lightSensor_readFilteredLuminosityValue(void)
{
    float raw = lightSensor_readRawLuminosity();
    float filtered = sensorManagement_hampelFilter(fLuminosityFilteredBuffer,
                                                   MOVING_AVERAGE_BUFFER_SIZE, raw, 2.0f);
    float avg = sensorManagement_movingAverage(fLuminosityFilteredBuffer, filtered);
    sensorManagement_isTresholdReached(LUMINOSITY, avg, fLuminosityTreshold);
    return avg;
}

void lightSensor_debug(void)
{
    LOG_INF("BH1750: lux=%d", (int)lightSensor_readFilteredLuminosityValue());
}
