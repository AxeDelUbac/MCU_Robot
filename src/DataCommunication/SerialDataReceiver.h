#ifndef SERIAL_DATA_RECEIVER_H
#define SERIAL_DATA_RECEIVER_H

#include "SerialDataStructure.h"

// API
void SerialDataReceiver_init(void);
void SerialDataReceiver_process(MonitoringPacket_t* outMonitoringPacket);
void SerialDataReceiver_debug(void);
void SerialDataReceiver_printStats(void);

#endif // SERIAL_DATA_RECEIVER_H
