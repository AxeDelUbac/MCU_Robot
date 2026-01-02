#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/gnss.h>


#define GNSS_DT_NODE DT_NODELABEL(usart1)


LOG_MODULE_REGISTER(g0b1re, LOG_LEVEL_INF);


    /* LSM6 and MMC56X3 devices will be looked up by label from the DT overlay */
    const struct device *lsm6 = NULL;
    const struct device *mmc56 = NULL;


void imu_task(void *p1, void *p2, void *p3)
{
    LOG_INF("IMU task start");
    /* get devices by label defined in overlay */
    lsm6 = device_get_binding("LSM6");
    mmc56 = device_get_binding("MMC56X3");
    
    if (lsm6 == NULL || !device_is_ready(lsm6)) {
        LOG_ERR("LSM6 device not found or not ready");
    }
    /* No LIS3MDL configured in this build */
    if (mmc56 == NULL || !device_is_ready(mmc56)) {
        LOG_INF("MMC56X3 device not found or not ready");
    } 

    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value magn[3];

    while (1) {
        int rc;
        if (lsm6) {
            rc = sensor_sample_fetch(lsm6);
            // LOG_INF("LSM6 sensor_sample_fetch rc=%d", rc);
            if (rc == 0) {
                int rc2 = sensor_channel_get(lsm6, SENSOR_CHAN_ACCEL_XYZ, accel);
                if (rc2 != 0) {
                    LOG_WRN("LSM6 sensor_channel_get accel rc=%d", rc2);
                }
                rc2 = sensor_channel_get(lsm6, SENSOR_CHAN_GYRO_XYZ, gyro);
                if (rc2 != 0) {
                    LOG_WRN("LSM6 sensor_channel_get gyro rc=%d", rc2);
                }

                LOG_INF("LSM6: ACC x=%d.%06d y=%d.%06d z=%d.%06d",
                        accel[0].val1, (int)abs(accel[0].val2),
                        accel[1].val1, (int)abs(accel[1].val2),
                        accel[2].val1, (int)abs(accel[2].val2));

                LOG_INF("LSM6: GYR x=%d.%06d y=%d.%06d z=%d.%06d",
                        gyro[0].val1, (int)abs(gyro[0].val2),
                        gyro[1].val1, (int)abs(gyro[1].val2),
                        gyro[2].val1, (int)abs(gyro[2].val2));
            } else {
                LOG_DBG("Skipping LSM6 channel read due to fetch rc=%d", rc);
            }
        }

        if (mmc56) {
            rc = sensor_sample_fetch(mmc56);
            // LOG_INF("MMC56X3 sensor_sample_fetch rc=%d", rc);
            if (rc == 0) {
                int rc2 = sensor_channel_get(mmc56, SENSOR_CHAN_MAGN_XYZ, magn);
                if (rc2 != 0) {
                    LOG_WRN("MMC56X3 sensor_channel_get magn rc=%d", rc2);
                } else {

                    LOG_INF("MMC56X3: MAG x=%d.%06d y=%d.%06d z=%d.%06d",
                        magn[0].val1, (int)abs(magn[0].val2),
                             magn[1].val1, (int)abs(magn[1].val2),
                             magn[2].val1, (int)abs(magn[2].val2));
                }
            } else {
                LOG_DBG("Skipping MMC56X3 channel read due to fetch rc=%d", rc);
            }
        }

        k_sleep(K_SECONDS(0.1));
    }
}


/* message queue to forward GNSS data from callback to task */
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

/* Register callback for DT node 'gnss-nmea-generic' (we added this in overlay) */
GNSS_DT_DATA_CALLBACK_DEFINE(DT_NODELABEL(neo_7m), gnss_data_cb);
K_THREAD_DEFINE(imu_id, 1024, imu_task, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(gnss_id, 1024, gnss_task, NULL, NULL, NULL, 4, 0, 0);

int main(void)
{
    LOG_INF("Booting Zephyr OS\n");

    // if (!gpio_is_ready_dt(&led)) {
    //     LOG_ERR("LED GPIO device not ready (devicetree node?)");
    //     return -1;
    // }

    // int rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    // LOG_INF("gpio_pin_configure_dt -> %d", rc);
    // if (rc != 0) {
    //     LOG_ERR("Failed to configure LED pin");
    //     return -1;
    // }

    // gpio_is_ready_dt(&led);
    // gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    // k_thread_start(task1_id);
    // k_thread_start(task2_id);
    // k_thread_start(bme_id);

    return 0;
}
