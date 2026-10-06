#include "Imu.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

static const struct device *const lsm6_dev =
    DEVICE_DT_GET(DT_NODELABEL(iks4a1_imu));
static const struct device *const lis2mdl_dev =
    DEVICE_DT_GET(DT_NODELABEL(iks4a1_magn));

ImuData_t        imuData;
ImuOrientation_t imuOrientation;
static K_MUTEX_DEFINE(imu_data_mutex);

static inline float sensor_val_to_float(const struct sensor_value *v)
{
    return (float)v->val1 + (float)v->val2 * 1e-6f;
}

void Imu_init(void)
{
    if (!device_is_ready(lsm6_dev)) {
        LOG_ERR("X-NUCLEO-IKS4A1 LSM6DSO16IS not ready");
    } else {
        LOG_DBG("X-NUCLEO-IKS4A1 LSM6DSO16IS ready");
    }
    if (!device_is_ready(lis2mdl_dev)) {
        LOG_ERR("X-NUCLEO-IKS4A1 LIS2MDL not ready");
    } else {
        LOG_DBG("X-NUCLEO-IKS4A1 LIS2MDL ready");
    }

    k_sleep(K_MSEC(50));
}

void Imu_getAccelerometer(void)
{
    if (!device_is_ready(lsm6_dev)) return;
    struct sensor_value val[3];
    int ret = sensor_sample_fetch(lsm6_dev);
    if (ret != 0) {
        LOG_WRN("LSM6DSO16IS sample fetch failed: %d", ret);
        return;
    }

    ret = sensor_channel_get(lsm6_dev, SENSOR_CHAN_ACCEL_XYZ, val);
    if (ret == 0) {
        k_mutex_lock(&imu_data_mutex, K_FOREVER);
        for (int i = 0; i < 3; i++) {
            imuData.fAcceleration_m_s2[i] = sensor_val_to_float(&val[i]);
            imuData.i32Acceleration[i]    = val[i].val1;
        }
        k_mutex_unlock(&imu_data_mutex);
    } else {
        LOG_WRN("LSM6DSO16IS accelerometer read failed: %d", ret);
    }

    ret = sensor_channel_get(lsm6_dev, SENSOR_CHAN_GYRO_XYZ, val);
    if (ret == 0) {
        k_mutex_lock(&imu_data_mutex, K_FOREVER);
        for (int i = 0; i < 3; i++) {
            imuData.fGyroscope_rad_s[i] = sensor_val_to_float(&val[i]);
            imuData.i32Gyroscope[i]     = val[i].val1;
        }
        k_mutex_unlock(&imu_data_mutex);
    } else {
        LOG_WRN("LSM6DSO16IS gyroscope read failed: %d", ret);
    }
}

void Imu_getGyroscope(void)
{
    /* data is already filled by Imu_getAccelerometer (shared LSM6 fetch) */
}

void Imu_getMagnetometer(void)
{
    if (!device_is_ready(lis2mdl_dev)) return;
    struct sensor_value val[3];
    int ret = sensor_sample_fetch(lis2mdl_dev);
    if (ret != 0) {
        LOG_WRN("LIS2MDL sample fetch failed: %d", ret);
        return;
    }
    ret = sensor_channel_get(lis2mdl_dev, SENSOR_CHAN_MAGN_XYZ, val);
    if (ret != 0) {
        LOG_WRN("LIS2MDL magnetometer read failed: %d", ret);
        return;
    }

    k_mutex_lock(&imu_data_mutex, K_FOREVER);
    for (int i = 0; i < 3; i++) {
        /* Zephyr's LIS2MDL driver reports magnetic flux density in µT. */
        imuData.fMagnetometer_uT[i] = sensor_val_to_float(&val[i]);
        imuData.i32Magnetometer[i]  = val[i].val1;
    }
    k_mutex_unlock(&imu_data_mutex);
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

    ImuData_t data;
    Imu_getTelemetrySnapshot(&data, NULL);
    float gx = data.fGyroscope_rad_s[0];
    float gy = data.fGyroscope_rad_s[1];
    float gz = data.fGyroscope_rad_s[2];

    /* Simple gyro integration — replace with complementary/Madgwick filter later */
    k_mutex_lock(&imu_data_mutex, K_FOREVER);
    imuOrientation.froll  += gx * dt;
    imuOrientation.fpitch += gy * dt;
    imuOrientation.fyaw   += gz * dt;
    k_mutex_unlock(&imu_data_mutex);
}

void Imu_getTelemetrySnapshot(ImuData_t *data, ImuOrientation_t *orientation)
{
    k_mutex_lock(&imu_data_mutex, K_FOREVER);
    if (data != NULL) {
        *data = imuData;
    }
    if (orientation != NULL) {
        *orientation = imuOrientation;
    }
    k_mutex_unlock(&imu_data_mutex);
}

void Imu_SerialDebug(void)
{
    ImuData_t data;
    ImuOrientation_t orientation;
    Imu_getTelemetrySnapshot(&data, &orientation);

    /* Scale to integer units to avoid float formatting on Cortex-M0+. */
    printk("IMU acc=[%d,%d,%d] mm/s2 gyro=[%d,%d,%d] urad/s "
            "mag=[%d,%d,%d] nT euler=[%d,%d,%d] urad",
        (int)(data.fAcceleration_m_s2[0] * 1000),
        (int)(data.fAcceleration_m_s2[1] * 1000),
        (int)(data.fAcceleration_m_s2[2] * 1000),
        (int)(data.fGyroscope_rad_s[0] * 1000000),
        (int)(data.fGyroscope_rad_s[1] * 1000000),
        (int)(data.fGyroscope_rad_s[2] * 1000000),
        (int)(data.fMagnetometer_uT[0] * 1000),
        (int)(data.fMagnetometer_uT[1] * 1000),
        (int)(data.fMagnetometer_uT[2] * 1000),
        (int)(orientation.froll  * 1000000),
        (int)(orientation.fpitch * 1000000),
        (int)(orientation.fyaw   * 1000000));
    printk("\r\n");
}
