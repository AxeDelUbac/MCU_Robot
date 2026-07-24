#include "sensorManagement.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

static const char *const sensor_names[] = {
    "TEMPERATURE", "HUMIDITY", "PRESSURE", "IAQ", "LUMINOSITY"
};

float sensorManagement_movingAverage(float *fSensorBuffer, float fNewValue)
{
    float sum = 0;
    int valid = 0;

    for (int i = MOVING_AVERAGE_BUFFER_SIZE - 1; i > 0; i--) {
        fSensorBuffer[i] = fSensorBuffer[i - 1];
    }
    fSensorBuffer[0] = fNewValue;

    for (int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++) {
        if (fSensorBuffer[i] != 0.0f) {
            sum += fSensorBuffer[i];
            valid++;
        }
    }
    return (valid > 0) ? (sum / valid) : 0.0f;
}

float sensorManagement_hampelFilter(float *fHampelSensorBuffer, int fHampelBufferSize,
                                    float fBufferNewValue, float threshold)
{
    int n = fHampelBufferSize + 1;
    float temp[n];

    for (int i = 0; i < fHampelBufferSize; i++) {
        temp[i] = fHampelSensorBuffer[i];
    }
    temp[fHampelBufferSize] = fBufferNewValue;

    /* Insertion sort to find median */
    for (int i = 1; i < n; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    float median = (n % 2 == 0)
        ? (temp[n / 2 - 1] + temp[n / 2]) / 2.0f
        : temp[n / 2];

    /* Compute absolute deviations and sort them */
    float dev[n];
    for (int i = 0; i < n; i++) {
        dev[i] = fabsf(temp[i] - median);
    }
    for (int i = 1; i < n; i++) {
        float key = dev[i];
        int j = i - 1;
        while (j >= 0 && dev[j] > key) {
            dev[j + 1] = dev[j];
            j--;
        }
        dev[j + 1] = key;
    }
    float mad = dev[n / 2];

    if (mad > 0.0f && fabsf(fBufferNewValue - median) > threshold * mad) {
        return median;
    }
    return fBufferNewValue;
}

void sensorManagement_isTresholdReached(eSensorType sensorType, float fSensorValue,
                                        float fThreshold[2])
{
    if (fSensorValue < fThreshold[0] || fSensorValue > fThreshold[1]) {
        LOG_WRN("Threshold exceeded: %s = %d (min=%d max=%d)",
            sensor_names[sensorType],
            (int)fSensorValue,
            (int)fThreshold[0],
            (int)fThreshold[1]);
    }
}
