#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include "RTOS.h"
#include "positionManagement/PositionOrientation.h"

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

void MotorRegulationTask(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sleep(K_MSEC(10));
    }
}

void speedMesurementTask(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sleep(K_MSEC(10));
    }
}

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

        k_sleep(K_MSEC(100));
    }
}

/* GNSS */
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

K_THREAD_DEFINE(motor_regulation_id, 2048, MotorRegulationTask, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(speed_mesurement_id, 1024, speedMesurementTask,  NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(imu_task_id,         1024, IMUTask,              NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(gnss_id,             1024, gnss_task,            NULL, NULL, NULL, 4, 0, 0);

