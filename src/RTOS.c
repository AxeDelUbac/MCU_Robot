#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include <string.h>
#include "RTOS.h"
#include "positionManagement/PositionOrientation.h"
#include "sensorManagement/environmentalSensor.h"
#include "sensorManagement/lightSensor.h"
#include "speedMesurement/GlobalSpeed.h"
#include "speedControlSystem/GlobalControl.h"
#include "navigation/forwardKinematics.h"
#include "navigation/mecanumOdometrie.h"
#include "DataCommunication/SerialDataTransmitter.h"
#include "DataCommunication/SerialDataReceiver.h"

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

/* Vitesses mesurées partagées entre speedMesurementTask et MotorRegulationTask */
static K_MUTEX_DEFINE(speed_mutex);
static float fMesuredWheelSpeed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* Dernière commande de mouvement reçue du SBC Linux, partagée entre
 * DataCommunicationTask et MotorRegulationTask */
static K_MUTEX_DEFINE(command_mutex);
static MonitoringPacket_t tRemoteCommand = {0};

/* ── Régulation moteur (20 Hz) ──────────────────────────────────────── */
void MotorRegulationTask(void *p1, void *p2, void *p3)
{
    static GlobalControl       tGlobalControl;
    static forwardKinematics_t tForwardKinematics;
    static MecanumOdometry_t   tOdometry;

    GlobalControl_init(&tGlobalControl);

    tForwardKinematics.vx            = 0;
    tForwardKinematics.vy            = 0;
    tForwardKinematics.omega         = 0;
    tForwardKinematics.geometricFactor = 1.0f; /* à ajuster : (L+W)/2 en unités normalisées */

    MecanumOdometry_init(&tOdometry, 0.039f, 0.20f, 0.18f); /* rayon roue, wheelBase, trackWidth */

    float fSetpointNorm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float fPIDOutput[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
    float fMesuredLocal[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    while (1) {
        /* Cinématique directe : consigne vx/vy/omega reçue du SBC Linux → vitesses normalisées roues */
        k_mutex_lock(&command_mutex, K_FOREVER);
        tForwardKinematics.vx    = tRemoteCommand.dirX;
        tForwardKinematics.vy    = tRemoteCommand.dirY;
        tForwardKinematics.omega = tRemoteCommand.omega;
        k_mutex_unlock(&command_mutex);

        ForwardKinematics_computeWheelVelocities(&tForwardKinematics, fSetpointNorm);

        /* Lecture thread-safe des vitesses mesurées par les encodeurs */
        k_mutex_lock(&speed_mutex, K_FOREVER);
        for (int i = 0; i < 4; i++) fMesuredLocal[i] = fMesuredWheelSpeed[i];
        k_mutex_unlock(&speed_mutex);

        /* Asservissement PID */
        GlobalControl_UpdateSetpoint(&tGlobalControl, fSetpointNorm, fMesuredLocal, fPIDOutput);

        /* TODO: MovementController_setMovement(fPIDOutput) → commande PWM moteurs */

        /* Odométrie */
        MecanumOdometry_updateWheelSpeeds(&tOdometry, fMesuredLocal);

        k_sleep(K_MSEC(50)); /* 20 Hz */
    }
}

/* ── Mesure vitesse encodeurs (10 Hz) ───────────────────────────────── */
void speedMesurementTask(void *p1, void *p2, void *p3)
{
    GlobalSpeed_init();

    float speeds[4];
    while (1) {
        k_sleep(K_MSEC(SPEED_MESUREMENT_PERIOD_MS));
        GlobalSpeed_getMeanSpeedInRPM(speeds, SPEED_MESUREMENT_PERIOD_MS);

        k_mutex_lock(&speed_mutex, K_FOREVER);
        for (int i = 0; i < 4; i++) fMesuredWheelSpeed[i] = speeds[i];
        k_mutex_unlock(&speed_mutex);

        GlobalSpeed_debug();
    }
}

/* ── IMU (configurable) ─────────────────────────────────────────────── */
void IMUTask(void *p1, void *p2, void *p3)
{
    PositionOrientation_init();

    int64_t last_ms = k_uptime_get();
    while (1) {
        int64_t now_ms = k_uptime_get();
        float dt = (float)(now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;

        PositionOrientation_update(dt);
        Imu_SerialDebug();

        k_sleep(K_MSEC(2000));
    }
}

/* ── GNSS ───────────────────────────────────────────────────────────── */
K_MSGQ_DEFINE(gnss_msgq, sizeof(struct gnss_data), 8, 4);

void gnss_task(void *a, void *b, void *c)
{
    struct gnss_data sample;
    while (1) {
        if (k_msgq_get(&gnss_msgq, &sample, K_FOREVER) == 0) {
            LOG_INF("NEO7M: GPS lat=%lld lon=%lld alt_mm=%d fix=%d sats=%d",
                (long long)sample.nav_data.latitude,
                (long long)sample.nav_data.longitude,
                sample.nav_data.altitude,
                sample.info.fix_status,
                sample.info.satellites_cnt);
        }
    }
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
    if (!data) { return; }
    if (k_msgq_put(&gnss_msgq, (void *)data, K_NO_WAIT) != 0) {
        LOG_WRN("GNSS msgq full, dropping sample");
    }
}

GNSS_DT_DATA_CALLBACK_DEFINE(DT_NODELABEL(neo_7m), gnss_data_cb);

/* ── Capteurs environnementaux + lumière ────────────────────────────── */
void sensorTask(void *p1, void *p2, void *p3)
{
    environmentalSensor_begin();
    lightSensor_begin();

    while (1) {
        environmentalSensor_debug();
        lightSensor_debug();
        k_sleep(K_MSEC(100));
    }
}

/* ── Liaison série vers le SBC Linux embarqué (DataCommunication, 20 Hz) ── */
void DataCommunicationTask(void *p1, void *p2, void *p3)
{
    SerialDataTransmitter_init();
    SerialDataReceiver_init();

    MonitoringPacket_t rxCommand = {0};
    MotorDataPacket_t  txMotorData = {0};
    float fSpeedLocal[4];

    while (1) {
        /* Réception de la commande de mouvement envoyée par le SBC Linux */
        SerialDataReceiver_process(&rxCommand);

        k_mutex_lock(&command_mutex, K_FOREVER);
        tRemoteCommand = rxCommand;
        k_mutex_unlock(&command_mutex);

        /* Télémétrie : vitesses roues mesurées renvoyées vers le SBC Linux */
        k_mutex_lock(&speed_mutex, K_FOREVER);
        for (int i = 0; i < 4; i++) fSpeedLocal[i] = fMesuredWheelSpeed[i];
        k_mutex_unlock(&speed_mutex);

        memcpy(txMotorData.wheelSpeedRPM, fSpeedLocal, sizeof(fSpeedLocal));
        /* TODO: renseigner txMotorData.motorPower depuis la sortie PID de MotorRegulationTask */
        SerialDataTransmitter_sendCompleteData(&txMotorData);

        k_sleep(K_MSEC(50)); /* 20 Hz */
    }
}

K_THREAD_DEFINE(motor_regulation_id, 2048, MotorRegulationTask, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(speed_mesurement_id, 1024, speedMesurementTask,  NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(imu_task_id,         1024, IMUTask,              NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(gnss_id,             1024, gnss_task,            NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(sensor_task_id,      2048, sensorTask,           NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(data_comm_task_id,   1024, DataCommunicationTask, NULL, NULL, NULL, 4, 0, 0);
