#include "SerialDataReceiver.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

/* TEMPORAIRE : USART1 (connecteur Arduino D0/D1) au lieu d'USART3, le temps des tests */
static const struct device *const s_uartDev = DEVICE_DT_GET(DT_NODELABEL(usart1));

RING_BUF_DECLARE(s_rxRingBuf, 256);

// Lecture octet-par-octet : machine à états
typedef enum {
    UART_RECEIVE_WAIT_START_STATE = 0,
    UART_RECEIVE_READ_LENGTH_STATE,
    UART_RECEIVE_READ_TYPE_STATE,
    UART_RECEIVE_READ_DATA_STATE,
    UART_RECEIVE_READ_CHECKSUM_STATE
} RecvState_t;

static RecvState_t s_state = UART_RECEIVE_WAIT_START_STATE;
static uint8_t s_expectedLength = 0;
static uint8_t s_packetType = 0;
static uint8_t s_dataBuf[256];
static uint8_t s_dataIdx = 0;

// Statistiques pour diagnostic
static uint32_t s_statsFramesComplete = 0;
static uint32_t s_statsFramesMonitoring = 0;
static uint32_t s_statsFramesMotor = 0;
static uint32_t s_statsFramesUnknown = 0;
static uint32_t s_statsBytesIgnored = 0;

// Appelé en contexte ISR : vide le FIFO matériel vers le ring buffer
static void SerialDataReceiver_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t byte;

    uart_irq_update(dev);
    while (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &byte, 1) == 1) {
            ring_buf_put(&s_rxRingBuf, &byte, 1);
        }
        uart_irq_update(dev);
    }
}

void SerialDataReceiver_init(void)
{
    if (!device_is_ready(s_uartDev)) {
        LOG_ERR("DataCommunication RX UART not ready");
        return;
    }

    s_state = UART_RECEIVE_WAIT_START_STATE;
    s_expectedLength = 0;
    s_packetType = 0;
    s_dataIdx = 0;

    uart_irq_callback_user_data_set(s_uartDev, SerialDataReceiver_isr, NULL);
    uart_irq_rx_enable(s_uartDev);

    LOG_INF("DataCommunication RX UART ready");
}

// A appeler régulièrement (depuis une tâche) : dépile le ring buffer et traite les trames
void SerialDataReceiver_process(MonitoringPacket_t* outMonitoringPacket)
{
    uint8_t b;

    while (ring_buf_get(&s_rxRingBuf, &b, 1) == 1) {
        switch (s_state)
        {
            case UART_RECEIVE_WAIT_START_STATE:
                if (b == SERIAL_START_BYTE) {
                    s_state = UART_RECEIVE_READ_LENGTH_STATE;
                } else {
                    s_statsBytesIgnored++;
                }
            break;
            case UART_RECEIVE_READ_LENGTH_STATE:
                s_expectedLength = b;
                s_state = UART_RECEIVE_READ_TYPE_STATE;
            break;
            case UART_RECEIVE_READ_TYPE_STATE:
                s_packetType = b;
                s_dataIdx = 0;
                s_state = (s_expectedLength == 0) ? UART_RECEIVE_READ_CHECKSUM_STATE
                                                   : UART_RECEIVE_READ_DATA_STATE;
            break;
            case UART_RECEIVE_READ_DATA_STATE:
                if (s_dataIdx < sizeof(s_dataBuf)) {
                    s_dataBuf[s_dataIdx++] = b;
                } else {
                    // overflow -> reset
                    s_state = UART_RECEIVE_WAIT_START_STATE;
                    s_dataIdx = 0;
                    break;
                }
                if (s_dataIdx >= s_expectedLength) {
                    s_state = UART_RECEIVE_READ_CHECKSUM_STATE;
                }
            break;
            case UART_RECEIVE_READ_CHECKSUM_STATE:
            {
                // On lit le checksum mais on ne le vérifie pas (implémentation future)
                uint8_t receivedChecksum = b;
                (void)receivedChecksum;

                LOG_DBG("Frame len=%d type=0x%02X", s_expectedLength, s_packetType);
                LOG_HEXDUMP_DBG(s_dataBuf, s_expectedLength, "RX payload");

                s_statsFramesComplete++;
                if (s_packetType == PACKET_TYPE_MONITORING) {
                    s_statsFramesMonitoring++;
                    if (s_expectedLength >= sizeof(MonitoringPacket_t)) {
                        memcpy(outMonitoringPacket, s_dataBuf, sizeof(MonitoringPacket_t));

                        LOG_DBG("MonPkt: dirX=%d dirY=%d omega=%d speedRatio=%d",
                            outMonitoringPacket->dirX, outMonitoringPacket->dirY,
                            outMonitoringPacket->omega, outMonitoringPacket->speedRatio);
                    }
                } else if (s_packetType == PACKET_TYPE_COMPLETE_DATA) {
                    s_statsFramesMotor++;
                    LOG_DBG("Paquet de données moteur reçu (non traité pour monitoring)");
                } else {
                    s_statsFramesUnknown++;
                    LOG_WRN("Type de paquet inconnu: 0x%02X", s_packetType);
                }

                // reset machine
                s_state = UART_RECEIVE_WAIT_START_STATE;
                s_dataIdx = 0;
                s_expectedLength = 0;
                s_packetType = 0;
            }
            break;
            default:
                s_state = UART_RECEIVE_WAIT_START_STATE;
            break;
        }
    }
}

void SerialDataReceiver_debug(void)
{
    static const char *const stateNames[] = {
        "WAIT_START", "READ_LENGTH", "READ_TYPE", "READ_DATA", "READ_CHECKSUM"
    };
    LOG_INF("État RX: %s", stateNames[s_state]);
}

void SerialDataReceiver_printStats(void)
{
    LOG_INF("STATS RX: complet=%u monitoring=%u moteur=%u inconnu=%u ignorés=%u",
        s_statsFramesComplete, s_statsFramesMonitoring, s_statsFramesMotor,
        s_statsFramesUnknown, s_statsBytesIgnored);
}
