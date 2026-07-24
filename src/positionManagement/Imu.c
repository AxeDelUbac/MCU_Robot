#include "Imu.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

static const struct device *lsm6_dev;
static const struct device *mmc56_dev;

ImuData_t        imuData;
ImuOrientation_t imuOrientation;

static inline float sensor_val_to_float(const struct sensor_value *v)
{
    return (float)v->val1 + (float)v->val2 * 1e-6f;
}

void Imu_init(void)
{
    lsm6_dev  = device_get_binding("LSM6");
    mmc56_dev = device_get_binding("MMC56X3");

    if (!lsm6_dev || !device_is_ready(lsm6_dev)) {
        LOG_ERR("LSM6 not ready");
    } else {
        LOG_INF("LSM6 ready");
    }
    if (!mmc56_dev || !device_is_ready(mmc56_dev)) {
        LOG_WRN("MMC56X3 not ready");
    } else {
        LOG_INF("MMC56X3 ready");
    }

    k_sleep(K_MSEC(50));
}

void Imu_getAccelerometer(void)
{
    if (!lsm6_dev) return;
    struct sensor_value val[3];
    if (sensor_sample_fetch(lsm6_dev) != 0) return;

    if (sensor_channel_get(lsm6_dev, SENSOR_CHAN_ACCEL_XYZ, val) == 0) {
        for (int i = 0; i < 3; i++) {
            imuData.fAcceleration_m_s2[i] = sensor_val_to_float(&val[i]);
            imuData.i32Acceleration[i]    = val[i].val1;
        }
    }
    /* fetch brings both accel and gyro — read gyro while we're here */
    if (sensor_channel_get(lsm6_dev, SENSOR_CHAN_GYRO_XYZ, val) == 0) {
        for (int i = 0; i < 3; i++) {
            imuData.fGyroscope_rad_s[i] = sensor_val_to_float(&val[i]);
            imuData.i32Gyroscope[i]     = val[i].val1;
        }
    }
}

void Imu_getGyroscope(void)
{
    /* data is already filled by Imu_getAccelerometer (shared LSM6 fetch) */
}

void Imu_getMagnetometer(void)
{
    if (!mmc56_dev) return;
    struct sensor_value val[3];
    if (sensor_sample_fetch(mmc56_dev) != 0) return;
    if (sensor_channel_get(mmc56_dev, SENSOR_CHAN_MAGN_XYZ, val) != 0) return;

    for (int i = 0; i < 3; i++) {
        /* MMC56X3 driver returns Gauss; 1 Gauss = 100 µT */
        imuData.fMagnetometer_uT[i] = sensor_val_to_float(&val[i]) * 100.0f;
        imuData.i32Magnetometer[i]  = val[i].val1;
    }
}

void Imu_getAcceleration_m_s2(void)
{
    Imu_getAccelerometer();
}

void Imu_getAngularRate_deg_s(float out_deg_s[3])
{
    for (int i = 0; i < 3; i++) {
        out_deg_s[i] = imuData.fGyroscope_rad_s[i] * (180.0f / 3.14159265f);
    }
}

void Imu_getAngularRate_rad_s(void)
{
    /* data already updated by Imu_getAccelerometer */
}

void Imu_updateOrientation(float dt)
{
    Imu_getAccelerometer();
    Imu_getMagnetometer();

    float gx = imuData.fGyroscope_rad_s[0];
    float gy = imuData.fGyroscope_rad_s[1];
    float gz = imuData.fGyroscope_rad_s[2];

    /* Simple gyro integration — replace with complementary/Madgwick filter later */
    imuOrientation.froll  += gx * dt;
    imuOrientation.fpitch += gy * dt;
    imuOrientation.fyaw   += gz * dt;
}

void Imu_SerialDebug(void)
{
    /* Scale to integer milli-units to avoid float printing on Cortex-M0+ */
    LOG_INF("ACC x=%d y=%d z=%d [mm/s2]",
        (int)(imuData.fAcceleration_m_s2[0] * 1000),
        (int)(imuData.fAcceleration_m_s2[1] * 1000),
        (int)(imuData.fAcceleration_m_s2[2] * 1000));
    LOG_INF("GYR x=%d y=%d z=%d [urad/s]",
        (int)(imuData.fGyroscope_rad_s[0] * 1000000),
        (int)(imuData.fGyroscope_rad_s[1] * 1000000),
        (int)(imuData.fGyroscope_rad_s[2] * 1000000));
    LOG_INF("MAG x=%d y=%d z=%d [nT]",
        (int)(imuData.fMagnetometer_uT[0] * 1000),
        (int)(imuData.fMagnetometer_uT[1] * 1000),
        (int)(imuData.fMagnetometer_uT[2] * 1000));
    LOG_INF("ORI roll=%d pitch=%d yaw=%d [urad]",
        (int)(imuOrientation.froll  * 1000000),
        (int)(imuOrientation.fpitch * 1000000),
        (int)(imuOrientation.fyaw   * 1000000));
}
