#include "lightSensor.h"

#define MOVING_AVERAGE_BUFFER_SIZE 6

BH1750 bh1750;

float fLuminosityValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fLastValueLuminosity = 0;
float fAverageLuminosity = 0;

float fLuminosityTreshold[2] = {0.0, 2500.0};

float fLuminosityFilteredValueBuffer[MOVING_AVERAGE_BUFFER_SIZE] = {0};
float fFilteredLastValueLuminosity = 0;
float fFilteredAverageLuminosity = 0;

/****
 * @brief Initializes the BH1750 light sensor.
 *
 * This function attempts to start the BH1750 sensor at I2C address 0x23.
 * If the sensor is not detected, it prints an error message to the serial port
 * and enters an infinite loop to block execution.
 *
 * @note Prints a message to the serial port on success or failure.
 ****/
void lightSensor_begin(void)
{
    if (!bh1750.begin())
    {
        Serial.println("BH1750 non détecté !");
        while (1)
            ; // Boucle infinie si le capteur n'est pas détecté
    }
    Serial.println("BH1750 détecté !");
}

/****
 * @brief Performs a luminosity reading and returns the moving average value.
 *
 * This function reads the luminosity from the BH1750 sensor, updates the moving average buffer,
 * and returns the calculated average luminosity.
 *
 * @return Averaged relative luminosity (in percent).
 ****/
float lightSensor_readLuminosityValue(void)
{
    fLastValueLuminosity = bh1750.readLightLevel();
    fAverageLuminosity = sensorManagement_movingAverage(fLuminosityFilteredValueBuffer, fLastValueLuminosity);
    sensorManagement_isTresholdReached(LUMINOSITY, fAverageLuminosity, fLuminosityTreshold);
    return fAverageLuminosity;
}

/**
 * @brief Reads the raw luminosity value from the BH1750 sensor.
 * @return Raw luminosity (in lux).
 */
float lightSensor_readRawLuminosity(void)
{
    return bh1750.readLightLevel();
}

float lightSensor_readFilteredLuminosityValue(void)
{
    fFilteredLastValueLuminosity = bh1750.readLightLevel();
    fFilteredAverageLuminosity = sensorManagement_movingAverage(fLuminosityFilteredValueBuffer, fFilteredLastValueLuminosity);
    sensorManagement_isTresholdReached(LUMINOSITY, fFilteredAverageLuminosity, fLuminosityTreshold);
    return fFilteredAverageLuminosity;
}

/****
 * END OF FILE
 ****/