#include "environmentalSensor.h"

#define MOVING_AVERAGE_BUFFER_SIZE 6

Adafruit_BME680 bme(&Wire);

float fTemperatureValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fHumidityValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fPressureValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fGasValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};

float fTemperatureTreshold[2] = {-5.0, 35.0};
float fHumidityTreshold[2] = {0.0, 90.0};
float fPressureTreshold[2] = {300.0, 1100.0};
float fIAQTreshold[2] = {0.0, 480.0};

float fLastValueTemperature = 0;
float fLastValueHumidity = 0;
float fLastValuePressure = 0;
float fLastValueGas = 0;

float fAverageTemperature = 0;
float fAverageHumidity = 0;
float fAveragePressure = 0;
float fAverageGasResistance = 0;
float fAverageIAQ= 0;

float fTemperatureFilteredValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fHumidityFilteredValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fPressureFilteredValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fGasFilteredValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};

float fFilteredLastValueTemperature = 0;
float fFilteredLastValueHumidity = 0;
float fFilteredLastValuePressure = 0;
float fFilteredLastValueGas = 0;

float fFilteredAverageTemperature = 0;
float fFilteredAverageHumidity = 0;
float fFilteredAveragePressure = 0;
float fFilteredAverageGasResistance = 0;
float fFilteredAverageIAQ= 0;

/**
 * @brief Initializes the BME680 sensor.
 *
 * Attempts to start the BME680 sensor at I2C address 0x76.
 * If the sensor is not detected, prints an error message to the serial port
 * and enters an infinite loop to block execution.
 *
 * @note Prints a message to the serial port on success or failure.
 */
void environmentalSensor_begin()
{
    if (!bme.begin(0x76))
    { // Essaie 0x77 si 0x76 ne marche pas
        Serial.println("BME680 non détecté !");
        while (1)
            ; // Boucle infinie si le capteur n'est pas détecté
    }
    Serial.println("BME680 détecté !");
}

/**
 * @brief Calculates IAQ (Indoor Air Quality) from gas resistance.
 *
 * Normalizes the resistance and returns an IAQ value between 0 (excellent) and 500 (very poor).
 *
 * @param fResistance Gas resistance value in Ohms.
 * @return IAQ value (0 = excellent, 500 = very poor).
 */
float environmentalSensor_IAQManagement(float fResistance)
{
    // Plage typique : 10 kΩ (mauvais air) à 500 kΩ (excellent air)
    float iaqNorm = (fResistance - 10000.0f) / (500000.0f - 10000.0f);

    // Limiter iaqNorm entre 0 et 1 sans constrain
    if (iaqNorm < 0.0f)
        iaqNorm = 0.0f;
    if (iaqNorm > 1.0f)
        iaqNorm = 1.0f;

    // Inverser l’indice : 0 = excellent, 1 = mauvais
    float iaq = (1.0f - iaqNorm) * 500.0f;
    return iaq; // 0 = excellent, 500 = très mauvais
}

/**
 * @brief Reads the temperature and returns the moving average value.
 *
 * Reads the temperature from the BME680 sensor, updates the moving average buffer,
 * and returns the calculated average temperature.
 *
 * @return Averaged temperature (in degrees Celsius).
 */
float environmentalSensor_readTemperatureValue(void)
{
    fLastValueTemperature = bme.readTemperature();
    fAverageTemperature = sensorManagement_movingAverage(fTemperatureValueBuffer, fLastValueTemperature);
    sensorManagement_isTresholdReached(TEMPERATURE, fAverageTemperature, fTemperatureTreshold);
    return fAverageTemperature;
}


/**
 * @brief Reads the humidity and returns the moving average value.
 *
 * Reads the humidity from the BME680 sensor, updates the moving average buffer,
 * and returns the calculated average humidity.
 *
 * @return Averaged relative humidity (in percent).
 */
float environmentalSensor_readHumidityValue(void)
{
    fLastValueHumidity = bme.readHumidity();
    fAverageHumidity = sensorManagement_movingAverage(fHumidityValueBuffer, fLastValueHumidity);
    sensorManagement_isTresholdReached(HUMIDITY, fAverageHumidity, fHumidityTreshold);
    return fAverageHumidity;
}

/**
 * @brief Reads the pressure and returns the moving average value.
 *
 * Reads the pressure from the BME680 sensor, updates the moving average buffer,
 * and returns the calculated average pressure.
 *
 * @return Averaged atmospheric pressure (in hPa).
 */
float environmentalSensor_readPressureValue(void)
{
    fLastValuePressure = bme.readPressure();
    fLastValuePressure = fLastValuePressure / 100.0f; // Convertion in hPa
    fAveragePressure = sensorManagement_movingAverage(fPressureValueBuffer, fLastValuePressure);
    sensorManagement_isTresholdReached(PRESSURE, fAveragePressure, fPressureTreshold);
    return fAveragePressure;
}

/**
 * @brief Reads the gas resistance and returns the moving average value.
 *
 * Reads the gas resistance from the BME680 sensor, updates the moving average buffer,
 * and returns the calculated average IAQ value.
 *
 * @return Averaged IAQ value (0 = excellent, 500 = very poor).
 */
float environmentalSensor_readGasValue(void)
{
    fLastValueGas = bme.readGas();
    fAverageGasResistance = sensorManagement_movingAverage(fGasValueBuffer, fLastValueGas);
    fAverageIAQ = environmentalSensor_IAQManagement(fAverageGasResistance);
    sensorManagement_isTresholdReached(IAQ, fAverageIAQ, fIAQTreshold);
    return fAverageIAQ;
}

/**
 * @brief Reads the raw temperature value from the BME680 sensor.
 * @return Raw temperature (in degrees Celsius).
 */
float environmentalSensor_readRawTemperature(void)
{
    return bme.readTemperature();
}

/**
 * @brief Reads the raw humidity value from the BME680 sensor.
 * @return Raw relative humidity (in percent).
 */
float environmentalSensor_readRawHumidity(void)
{
    return bme.readHumidity();
}

/**
 * @brief Reads the raw pressure value from the BME680 sensor.
 * @return Raw atmospheric pressure (in hPa).
 */
float environmentalSensor_readRawPressure(void)
{
    return bme.readPressure() / 100.0f; // Convert Pa to hPa
}

/**
 * @brief Reads the raw gas resistance value from the BME680 sensor.
 * @return Raw gas resistance (in Ohms).
 */
float environmentalSensor_readRawGas(void)
{
    float fRawGas = bme.readGas();
    return environmentalSensor_IAQManagement(fRawGas);
}

float environmentalSensor_readFilteredTemperatureValue(void)
{
    fFilteredLastValueTemperature = bme.readTemperature();
    float filteredValue = sensorManagement_hampelFilter(fTemperatureFilteredValueBuffer, MOVING_AVERAGE_BUFFER_SIZE, fFilteredLastValueTemperature, 2.0);
    fFilteredAverageTemperature = sensorManagement_movingAverage(fTemperatureFilteredValueBuffer, filteredValue);
    sensorManagement_isTresholdReached(TEMPERATURE, fFilteredAverageTemperature, fTemperatureTreshold);
    return fFilteredAverageTemperature;
}

float environmentalSensor_readFilteredHumidityValue(void)
{
    fFilteredLastValueHumidity = bme.readHumidity();
    float filteredValue = sensorManagement_hampelFilter(fHumidityValueBuffer, MOVING_AVERAGE_BUFFER_SIZE, fFilteredLastValueHumidity, 2.0);
    fFilteredAverageHumidity = sensorManagement_movingAverage(fHumidityValueBuffer, filteredValue);
    sensorManagement_isTresholdReached(HUMIDITY, fFilteredAverageHumidity, fHumidityTreshold);
    return fFilteredAverageHumidity;
}

float environmentalSensor_readFilteredPressureValue(void)
{
    fFilteredLastValuePressure = bme.readPressure()/100;// Convertion in hPa
    float filteredValue = sensorManagement_hampelFilter(fPressureValueBuffer, MOVING_AVERAGE_BUFFER_SIZE, fFilteredLastValuePressure, 2.0);
    fFilteredAveragePressure = sensorManagement_movingAverage(fPressureValueBuffer, filteredValue);
    sensorManagement_isTresholdReached(PRESSURE, fFilteredAveragePressure, fPressureTreshold);
    return fFilteredAveragePressure;
}

float environmentalSensor_readFilteredGasValue(void)
{
    fFilteredLastValueGas = bme.readGas();
    float filteredValue = sensorManagement_hampelFilter(fGasValueBuffer, MOVING_AVERAGE_BUFFER_SIZE, fFilteredLastValueGas, 2.0);
    fFilteredAverageGasResistance = sensorManagement_movingAverage(fGasValueBuffer, filteredValue);
    fFilteredAverageIAQ = environmentalSensor_IAQManagement(fFilteredAverageGasResistance);
    sensorManagement_isTresholdReached(IAQ, fFilteredAverageIAQ, fIAQTreshold);
    return fFilteredAverageIAQ;
}
/****
 * END OF FILE
 ****/