#include "environmentalSensor.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

static const struct device *bme680_dev;

static float fTemperatureBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fHumidityBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fPressureBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fGasBuffer[MOVING_AVERAGE_BUFFER_SIZE];

static float fTemperatureFilteredBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fHumidityFilteredBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fPressureFilteredBuffer[MOVING_AVERAGE_BUFFER_SIZE];
static float fGasFilteredBuffer[MOVING_AVERAGE_BUFFER_SIZE];

static float fTemperatureTreshold[2] = {-5.0f,  35.0f};
static float fHumidityTreshold[2]    = { 0.0f,  90.0f};
static float fPressureTreshold[2]    = {300.0f, 1100.0f};
static float fIAQTreshold[2]         = { 0.0f,  480.0f};

static float fIAQ(float resistance)
{
    /* Typical range: 10 kΩ (poor) to 500 kΩ (excellent); scale to 0–500 (lower = better) */
    float norm = (resistance - 10000.0f) / (500000.0f - 10000.0f);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return (1.0f - norm) * 500.0f;
}

static int bme680_fetch(struct sensor_value *temp, struct sensor_value *hum,
                        struct sensor_value *press, struct sensor_value *gas)
{
    if (!bme680_dev) return -1;
    if (sensor_sample_fetch(bme680_dev) != 0) return -1;
    sensor_channel_get(bme680_dev, SENSOR_CHAN_AMBIENT_TEMP, temp);
    sensor_channel_get(bme680_dev, SENSOR_CHAN_HUMIDITY,     hum);
    sensor_channel_get(bme680_dev, SENSOR_CHAN_PRESS,        press);
    sensor_channel_get(bme680_dev, SENSOR_CHAN_GAS_RES,      gas);
    return 0;
}

void environmentalSensor_begin(void)
{
    bme680_dev = device_get_binding("BME680");
    if (!bme680_dev || !device_is_ready(bme680_dev)) {
        LOG_ERR("BME680 not found or not ready");
        return;
    }
    LOG_INF("BME680 ready");
}

/* ── Raw reads ──────────────────────────────────────────────────── */

float environmentalSensor_readRawTemperature(void)
{
    struct sensor_value v, h, p, g;
    if (bme680_fetch(&v, &h, &p, &g) != 0) return 0.0f;
    return (float)v.val1 + (float)v.val2 * 1e-6f;
}

float environmentalSensor_readRawHumidity(void)
{
    struct sensor_value v, h, p, g;
    if (bme680_fetch(&v, &h, &p, &g) != 0) return 0.0f;
    return (float)h.val1 + (float)h.val2 * 1e-6f;
}

float environmentalSensor_readRawPressure(void)
{
    struct sensor_value v, h, p, g;
    if (bme680_fetch(&v, &h, &p, &g) != 0) return 0.0f;
    /* Zephyr returns kPa → convert to hPa (* 10) */
    return ((float)p.val1 + (float)p.val2 * 1e-6f) * 10.0f;
}

float environmentalSensor_readRawGas(void)
{
    struct sensor_value v, h, p, g;
    if (bme680_fetch(&v, &h, &p, &g) != 0) return 0.0f;
    float resistance = (float)g.val1 + (float)g.val2 * 1e-6f;
    return fIAQ(resistance);
}

/* ── Moving-average reads ───────────────────────────────────────── */

float environmentalSensor_readTemperatureValue(void)
{
    float raw = environmentalSensor_readRawTemperature();
    float avg = sensorManagement_movingAverage(fTemperatureBuffer, raw);
    sensorManagement_isTresholdReached(TEMPERATURE, avg, fTemperatureTreshold);
    return avg;
}

float environmentalSensor_readHumidityValue(void)
{
    float raw = environmentalSensor_readRawHumidity();
    float avg = sensorManagement_movingAverage(fHumidityBuffer, raw);
    sensorManagement_isTresholdReached(HUMIDITY, avg, fHumidityTreshold);
    return avg;
}

float environmentalSensor_readPressureValue(void)
{
    float raw = environmentalSensor_readRawPressure();
    float avg = sensorManagement_movingAverage(fPressureBuffer, raw);
    sensorManagement_isTresholdReached(PRESSURE, avg, fPressureTreshold);
    return avg;
}

float environmentalSensor_readGasValue(void)
{
    float raw = environmentalSensor_readRawGas();
    float avg = sensorManagement_movingAverage(fGasBuffer, raw);
    sensorManagement_isTresholdReached(IAQ, avg, fIAQTreshold);
    return avg;
}

/* ── Hampel + moving-average reads ─────────────────────────────── */

float environmentalSensor_readFilteredTemperatureValue(void)
{
    float raw = environmentalSensor_readRawTemperature();
    float filtered = sensorManagement_hampelFilter(fTemperatureFilteredBuffer,
                                                   MOVING_AVERAGE_BUFFER_SIZE, raw, 2.0f);
    float avg = sensorManagement_movingAverage(fTemperatureFilteredBuffer, filtered);
    sensorManagement_isTresholdReached(TEMPERATURE, avg, fTemperatureTreshold);
    return avg;
}

float environmentalSensor_readFilteredHumidityValue(void)
{
    float raw = environmentalSensor_readRawHumidity();
    float filtered = sensorManagement_hampelFilter(fHumidityFilteredBuffer,
                                                   MOVING_AVERAGE_BUFFER_SIZE, raw, 2.0f);
    float avg = sensorManagement_movingAverage(fHumidityFilteredBuffer, filtered);
    sensorManagement_isTresholdReached(HUMIDITY, avg, fHumidityTreshold);
    return avg;
}

float environmentalSensor_readFilteredPressureValue(void)
{
    float raw = environmentalSensor_readRawPressure();
    float filtered = sensorManagement_hampelFilter(fPressureFilteredBuffer,
                                                   MOVING_AVERAGE_BUFFER_SIZE, raw, 2.0f);
    float avg = sensorManagement_movingAverage(fPressureFilteredBuffer, filtered);
    sensorManagement_isTresholdReached(PRESSURE, avg, fPressureTreshold);
    return avg;
}

float environmentalSensor_readFilteredGasValue(void)
{
    float raw = environmentalSensor_readRawGas();
    float filtered = sensorManagement_hampelFilter(fGasFilteredBuffer,
                                                   MOVING_AVERAGE_BUFFER_SIZE, raw, 2.0f);
    float avg = sensorManagement_movingAverage(fGasFilteredBuffer, filtered);
    sensorManagement_isTresholdReached(IAQ, avg, fIAQTreshold);
    return avg;
}

void environmentalSensor_debug(void)
{
    LOG_INF("BME680: temp=%d°C hum=%d%% press=%dhPa IAQ=%d",
        (int)environmentalSensor_readFilteredTemperatureValue(),
        (int)environmentalSensor_readFilteredHumidityValue(),
        (int)environmentalSensor_readFilteredPressureValue(),
        (int)environmentalSensor_readFilteredGasValue());
}
