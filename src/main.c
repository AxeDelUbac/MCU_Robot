#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(g0b1re, LOG_LEVEL_INF);

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
