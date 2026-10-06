#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include "RTOS.h"
#include "positionManagement/PositionOrientation.h"
#include "sensorManagement/lps22dfSensor.h"
#include "speedMesurement/GlobalSpeed.h"
#include "speedControlSystem/GlobalControl.h"
#include "navigation/forwardKinematics.h"
#include "navigation/mecanumOdometrie.h"
#include "DataCommunication/MicroRosNode.h"

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

/* Vitesses mesurées partagées entre speedMesurementTask et MotorRegulationTask */
static K_MUTEX_DEFINE(speed_mutex);
static float fMesuredWheelSpeed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* Dernière commande Twist reçue par micro-ROS, partagée avec la régulation. */
static K_MUTEX_DEFINE(gnss_data_mutex);
static struct gnss_data latest_gnss_sample;
static bool gnss_sample_available;

static void gnss_debug_print_latest(void);

/* ── Robot communication task ───────────────────────────────────────── */
void RobotCommunicationTask(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MicroRosNode_run();
}

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
        /* Cinématique directe : consigne Twist du SBC → vitesses normalisées roues. */
        float linear_x;
        float linear_y;
        float angular_z;
        RobotCommunication_getCommand(&linear_x, &linear_y, &angular_z);
        tForwardKinematics.vx = (int)(linear_x * 100.0f);
        tForwardKinematics.vy = (int)(linear_y * 100.0f);
        tForwardKinematics.omega = (int)(angular_z * 100.0f);

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
    }
}

/* ── IMU (configurable) ─────────────────────────────────────────────── */
void IMUTask(void *p1, void *p2, void *p3)
{
    PositionOrientation_init();
    lps22dfSensor_init();

    int64_t last_ms = k_uptime_get();
    while (1) {
        int64_t now_ms = k_uptime_get();
        float dt = (float)(now_ms - last_ms) / 1000.0f;
        last_ms = now_ms;

        PositionOrientation_update(dt);
        Imu_SerialDebug();
        lps22dfSensor_debug();
        gnss_debug_print_latest();

        k_sleep(K_MSEC(2000));
    }
}

/* ── GNSS ───────────────────────────────────────────────────────────── */
K_MSGQ_DEFINE(gnss_msgq, sizeof(struct gnss_data), 8, 4);

static const struct device *const gnss_dev =
    DEVICE_DT_GET(DT_NODELABEL(teseo_liv3f));

static const char *gnss_fix_status_name(enum gnss_fix_status status)
{
    switch (status) {
    case GNSS_FIX_STATUS_NO_FIX:
        return "NO_FIX";
    case GNSS_FIX_STATUS_GNSS_FIX:
        return "GNSS_FIX";
    case GNSS_FIX_STATUS_DGNSS_FIX:
        return "DGNSS_FIX";
    case GNSS_FIX_STATUS_ESTIMATED_FIX:
        return "ESTIMATED";
    default:
        return "UNKNOWN";
    }
}

void gnss_task(void *a, void *b, void *c)
{
    if (!device_is_ready(gnss_dev)) {
        LOG_ERR("X-NUCLEO-GNSS1A1 Teseo-LIV3F not ready");
        return;
    }
    LOG_DBG("X-NUCLEO-GNSS1A1 Teseo-LIV3F ready; waiting for NMEA data");

    struct gnss_data sample;
    while (1) {
        int ret = k_msgq_get(&gnss_msgq, &sample, K_FOREVER);
        if (ret == 0) {
            k_mutex_lock(&gnss_data_mutex, K_FOREVER);
            latest_gnss_sample = sample;
            gnss_sample_available = true;
            k_mutex_unlock(&gnss_data_mutex);
        }
    }
}

bool RTOS_getGnssSample(struct gnss_data *sample)
{
    if (sample == NULL) {
        return false;
    }

    k_mutex_lock(&gnss_data_mutex, K_FOREVER);
    bool available = gnss_sample_available;
    if (available) {
        *sample = latest_gnss_sample;
    }
    k_mutex_unlock(&gnss_data_mutex);

    return available;
}

void RTOS_getWheelSpeeds(float speeds[4])
{
    if (speeds == NULL) {
        return;
    }

    k_mutex_lock(&speed_mutex, K_FOREVER);
    memcpy(speeds, fMesuredWheelSpeed, sizeof(fMesuredWheelSpeed));
    k_mutex_unlock(&speed_mutex);
}

static void gnss_debug_print_latest(void)
{
    struct gnss_data sample;
    bool available;

    k_mutex_lock(&gnss_data_mutex, K_FOREVER);
    sample = latest_gnss_sample;
    available = gnss_sample_available;
    k_mutex_unlock(&gnss_data_mutex);

    if (!available) {
        printk("GNSS: waiting for NMEA data\r\n");
        return;
    }

    int64_t latitude = sample.nav_data.latitude;
    int64_t longitude = sample.nav_data.longitude;
    uint64_t abs_latitude = latitude < 0
        ? (uint64_t)(-(latitude + 1)) + 1U : (uint64_t)latitude;
    uint64_t abs_longitude = longitude < 0
        ? (uint64_t)(-(longitude + 1)) + 1U : (uint64_t)longitude;

    printk("GNSS fix=%s quality=%d sats=%u "
        "lat=%s%llu.%09llu lon=%s%llu.%09llu deg alt=%d mm",
        gnss_fix_status_name(sample.info.fix_status),
        sample.info.fix_quality,
        sample.info.satellites_cnt,
        latitude < 0 ? "-" : "",
        (unsigned long long)(abs_latitude / 1000000000U),
        (unsigned long long)(abs_latitude % 1000000000U),
        longitude < 0 ? "-" : "",
        (unsigned long long)(abs_longitude / 1000000000U),
        (unsigned long long)(abs_longitude % 1000000000U),
        sample.nav_data.altitude);
    printk("\r\n");
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
    if (!data) { return; }
    if (k_msgq_put(&gnss_msgq, (void *)data, K_NO_WAIT) != 0) {
        LOG_WRN("GNSS msgq full, dropping sample");
    }
}

GNSS_DT_DATA_CALLBACK_DEFINE(DT_NODELABEL(teseo_liv3f), gnss_data_cb);

K_THREAD_DEFINE(motor_regulation_id, 2048, MotorRegulationTask, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(speed_mesurement_id, 1024, speedMesurementTask,  NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(imu_task_id,         1024, IMUTask,              NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(gnss_id,             1024, gnss_task,            NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(robot_communication_task_id, 8192, RobotCommunicationTask,
                NULL, NULL, NULL, 4, 0, 0);
