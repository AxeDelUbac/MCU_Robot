#include "SerialDataTransmitter.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

/* TEMPORAIRE : USART1 (connecteur Arduino D0/D1) au lieu d'USART3, le temps des tests */
static const struct device *const s_uartDev = DEVICE_DT_GET(DT_NODELABEL(usart1));

static void SerialDataTransmitter_write(const uint8_t* buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(s_uartDev, buf[i]);
    }
}

/**
 * @brief Initialise la liaison série vers le SBC Linux (UART configuré via devicetree)
 */
void SerialDataTransmitter_init(void)
{
    if (!device_is_ready(s_uartDev)) {
        LOG_ERR("DataCommunication TX UART not ready");
        return;
    }
    LOG_INF("DataCommunication TX UART ready");
}

/**
 * @brief Transmet les données moteurs (vitesses roues + puissance) en un seul paquet
 */
bool SerialDataTransmitter_sendCompleteData(MotorDataPacket_t* motorData)
{
    if (!device_is_ready(s_uartDev)) return false;

    motorData->motorID = MOTOR_DATA_PACKET_ID;

    uint8_t txBuffer[3 + sizeof(MotorDataPacket_t) + 1];
    size_t bufIdx = 0;

    txBuffer[bufIdx++] = SERIAL_START_BYTE;
    txBuffer[bufIdx++] = (uint8_t)sizeof(MotorDataPacket_t);
    txBuffer[bufIdx++] = PACKET_TYPE_COMPLETE_DATA;

    memcpy(&txBuffer[bufIdx], motorData, sizeof(MotorDataPacket_t));
    bufIdx += sizeof(MotorDataPacket_t);

    txBuffer[bufIdx++] = SerialDataTransmitter_calculateChecksum((uint8_t*)motorData, sizeof(MotorDataPacket_t));

    LOG_HEXDUMP_DBG(txBuffer, bufIdx, "TX");
    SerialDataTransmitter_write(txBuffer, bufIdx);

    return true;
}

/**
 * @brief Transmet un petit paquet de monitoring (dirX, dirY, omega, speedRatio)
 */
bool SerialDataTransmitter_sendMonitoringPacket(MonitoringPacket_t* monitoringData)
{
    if (!device_is_ready(s_uartDev)) return false;

    uint8_t txBuffer[3 + sizeof(MonitoringPacket_t) + 1];
    size_t bufIdx = 0;

    txBuffer[bufIdx++] = SERIAL_START_BYTE;
    txBuffer[bufIdx++] = (uint8_t)sizeof(MonitoringPacket_t);
    txBuffer[bufIdx++] = PACKET_TYPE_MONITORING;

    memcpy(&txBuffer[bufIdx], monitoringData, sizeof(MonitoringPacket_t));
    bufIdx += sizeof(MonitoringPacket_t);

    txBuffer[bufIdx++] = SerialDataTransmitter_calculateChecksum((uint8_t*)monitoringData, sizeof(MonitoringPacket_t));

    LOG_HEXDUMP_DBG(txBuffer, bufIdx, "TX");
    SerialDataTransmitter_write(txBuffer, bufIdx);

    return true;
}

/**
 * @brief Calcule un checksum simple (XOR) pour vérifier l'intégrité des données
 */
uint8_t SerialDataTransmitter_calculateChecksum(const uint8_t* data, size_t length)
{
    if (!data) return 0;

    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
