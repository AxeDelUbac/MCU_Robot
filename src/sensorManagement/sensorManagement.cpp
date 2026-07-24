#include "sensorManagement.h"

#define MOVING_AVERAGE_BUFFER_SIZE 6

// Buffers pour le moving average
// static float tempBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
// static float humBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
// static float pressBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
// static float gasBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
// static float lumiBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};

/**
 * @brief Calculates the moving average on a buffer and adds a new value.
 *
 * Shifts the values in the buffer, inserts the new value at the head,
 * and then calculates the average using only the non-null values present in the buffer.
 * This ensures that, at startup, the average is computed based on the actual number of valid values,
 * even if the buffer is not yet fully filled.
 *
 * @param fSensorBuffer Pointer to the buffer of float values.
 * @param fNewValue New value to insert into the buffer.
 * @return Moving average calculated on the buffer after inserting the new value.
 */
float sensorManagement_movingAverage(float *fSensorBuffer, float fNewValue)
{
    float AverageValue = 0;
    int iNumberOfZerosValues = 0;

    for (int iBcl = MOVING_AVERAGE_BUFFER_SIZE - 1; iBcl > 0; iBcl--)
    {
        fSensorBuffer[iBcl] = fSensorBuffer[iBcl - 1];
    }
    fSensorBuffer[0] = fNewValue;

    for (int iBcl = 0; iBcl < MOVING_AVERAGE_BUFFER_SIZE; iBcl++)
    {
        if(fSensorBuffer[iBcl] != 0.0)
        {
            AverageValue = AverageValue + fSensorBuffer[iBcl];
        }
        else
        {
            // if the value is zero, it means the buffer is not yet full
            iNumberOfZerosValues++;
        }
    }

    // Calculate average only on non-zero values
    AverageValue = AverageValue / (MOVING_AVERAGE_BUFFER_SIZE - iNumberOfZerosValues);
    return AverageValue;
}

float sensorManagement_hampelFilter(float *fHampelSensorBuffer, int fHampelBufferSize, float fBufferNewValue, float threshold)
{
    // Copie le buffer + nouvelle valeur
    float temp[fHampelBufferSize + 1];
    for (int i = 0; i < fHampelBufferSize; i++)
    {
        temp[i] = fHampelSensorBuffer[i];
    }
    temp[fHampelBufferSize] = fBufferNewValue;

    // Calcule la médiane
    float fmedian;
    for (int i = 1; i < fHampelBufferSize + 1; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    if (fHampelBufferSize % 2 == 0)
        fmedian = (temp[fHampelBufferSize / 2 - 1] + temp[fHampelBufferSize / 2]) / 2.0f;
    else
        fmedian = temp[fHampelBufferSize / 2];
    
    // Calcule la MAD
    float dev[fHampelBufferSize + 1];
    for (int i = 0; i < fHampelBufferSize + 1; i++) dev[i] = fabs(temp[i] - fmedian);
    std::sort(dev, dev + fHampelBufferSize + 1);
    float fMedianAbsoluteDeviation = dev[(fHampelBufferSize + 1) / 2];

    // Si la nouvelle valeur est trop éloignée, remplace par la médiane
    if (fMedianAbsoluteDeviation > 0 && fabs(fBufferNewValue - fmedian) > threshold * fMedianAbsoluteDeviation)
        return fmedian;
    else
        return fBufferNewValue;
}

/**
 * @brief Checks if a sensor value exceeds its defined threshold(s) and triggers an alert if needed.
 *
 * Compares the given sensor value to the provided thresholds (min and max).
 * If the value is outside the range, calls MQTTManagement_ThresholdReached to send an alert.
 *
 * @param sensorType The type of sensor being checked.
 * @param fSensorValue The current value measured by the sensor.
 * @param fThreshold Array of two floats: fThreshold[0] = min threshold, fThreshold[1] = max threshold.
 *                   Use -INFINITY or INFINITY if only one threshold is needed.
 * @return eSensorType The type of sensor if the threshold is exceeded, or -1 (casté) sinon.
 */
void sensorManagement_isTresholdReached(eSensorType sensorType, float fSensorValue, float fThreshold[2]) {
    if ((fSensorValue < fThreshold[0]) || (fSensorValue > fThreshold[1])) {
        MQTTManagement_ThresholdReached(sensorType, fSensorValue);
    }
}

/****
 * END OF FILE
 ****//**
 * @brief End of file marker for documentation purposes.
 */