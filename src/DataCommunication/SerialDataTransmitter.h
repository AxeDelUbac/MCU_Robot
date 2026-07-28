#ifndef SERIAL_DATA_TRANSMITTER_H
#define SERIAL_DATA_TRANSMITTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "SerialDataStructure.h"

// Fonctions d'initialisation
void SerialDataTransmitter_init(void);

// Fonctions de transmission
bool SerialDataTransmitter_sendCompleteData(MotorDataPacket_t* motorData);
bool SerialDataTransmitter_sendMonitoringPacket(MonitoringPacket_t* monitoringData);

// Fonctions utilitaires
uint8_t SerialDataTransmitter_calculateChecksum(const uint8_t* data, size_t length);

#endif // SERIAL_DATA_TRANSMITTER_H
