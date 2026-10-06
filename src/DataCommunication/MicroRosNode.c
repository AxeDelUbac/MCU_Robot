#include "MicroRosNode.h"

#include <geometry_msgs/msg/twist.h>
#include <builtin_interfaces/msg/time.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <uxr/client/transport.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <microros_transports.h>

#include "RTOS.h"
#include "positionManagement/Imu.h"
#include "sensorManagement/lps22dfSensor.h"

#define MICROROS_UART_RX_BUFFER_SIZE 512
#define MICROROS_SPIN_TIMEOUT_MS 10
#define MICROROS_COMMAND_TIMEOUT_MS 500
#define MICROROS_PUBLISH_PERIOD_MS 2000
#define ROBOT_MAX_LINEAR_SPEED_M_S 1.0f
#define ROBOT_MAX_ANGULAR_SPEED_RAD_S 1.0f

static const struct device *const microros_uart =
    DEVICE_DT_GET(DT_NODELABEL(usart3));
RING_BUF_DECLARE(microros_rx_ring, MICROROS_UART_RX_BUFFER_SIZE);

static K_MUTEX_DEFINE(command_mutex);
static float latest_cmd_linear_x;
static float latest_cmd_linear_y;
static float latest_cmd_angular_z;
static int64_t latest_cmd_timestamp_ms;
static bool command_received;

static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__MagneticField magnetic_field_msg;
static sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
static sensor_msgs__msg__FluidPressure pressure_msg;
static sensor_msgs__msg__Temperature temperature_msg;
static std_msgs__msg__Float32MultiArray wheel_speeds_msg;
static float wheel_speeds_data[4];
static geometry_msgs__msg__Twist cmd_vel_msg;

static void microros_uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    uart_irq_update(dev);
    while (uart_irq_rx_ready(dev)) {
        uint8_t bytes[32];
        int count = uart_fifo_read(dev, bytes, sizeof(bytes));
        if (count <= 0) {
            break;
        }
        ring_buf_put(&microros_rx_ring, bytes, (uint32_t)count);
        uart_irq_update(dev);
    }
}

static float clamp_speed(float value, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < -maximum) {
        return -maximum;
    }
    return value;
}

static bool microros_transport_open(struct uxrCustomTransport *transport)
{
    zephyr_transport_params_t *params = transport->args;
    if (!device_is_ready(microros_uart)) {
        return false;
    }

    ring_buf_reset(&microros_rx_ring);
    params->uart_dev = microros_uart;
    if (uart_irq_callback_user_data_set(params->uart_dev, microros_uart_isr, NULL) != 0) {
        params->uart_dev = NULL;
        return false;
    }
    uart_irq_rx_enable(params->uart_dev);
    return true;
}

static bool microros_transport_close(struct uxrCustomTransport *transport)
{
    zephyr_transport_params_t *params = transport->args;
    if (params->uart_dev != NULL) {
        uart_irq_rx_disable(params->uart_dev);
        uart_irq_callback_user_data_set(params->uart_dev, NULL, NULL);
        params->uart_dev = NULL;
    }
    return true;
}

static size_t microros_transport_write(struct uxrCustomTransport *transport,
                                       const uint8_t *buffer, size_t length,
                                       uint8_t *error)
{
    zephyr_transport_params_t *params = transport->args;
    ARG_UNUSED(error);

    if (params->uart_dev == NULL) {
        return 0;
    }
    for (size_t i = 0; i < length; i++) {
        uart_poll_out(params->uart_dev, buffer[i]);
    }
    return length;
}

static size_t microros_transport_read(struct uxrCustomTransport *transport,
                                      uint8_t *buffer, size_t length,
                                      int timeout_ms, uint8_t *error)
{
    zephyr_transport_params_t *params = transport->args;
    ARG_UNUSED(error);

    if (params->uart_dev == NULL || timeout_ms < 0) {
        return 0;
    }

    int64_t deadline = k_uptime_get() + timeout_ms;
    while (ring_buf_is_empty(&microros_rx_ring) && k_uptime_get() < deadline) {
        k_sleep(K_MSEC(1));
    }
    return ring_buf_get(&microros_rx_ring, buffer, (uint32_t)length);
}

void RobotCommunication_getCommand(float *linear_x, float *linear_y, float *angular_z)
{
    if (linear_x == NULL || linear_y == NULL || angular_z == NULL) {
        return;
    }

    k_mutex_lock(&command_mutex, K_FOREVER);
    bool command_is_fresh = command_received &&
        (k_uptime_get() - latest_cmd_timestamp_ms) <= MICROROS_COMMAND_TIMEOUT_MS;
    *linear_x = command_is_fresh ? latest_cmd_linear_x / ROBOT_MAX_LINEAR_SPEED_M_S : 0.0f;
    *linear_y = command_is_fresh ? latest_cmd_linear_y / ROBOT_MAX_LINEAR_SPEED_M_S : 0.0f;
    *angular_z = command_is_fresh
        ? latest_cmd_angular_z / ROBOT_MAX_ANGULAR_SPEED_RAD_S : 0.0f;
    k_mutex_unlock(&command_mutex);
}

static void cmd_vel_callback(const void *message)
{
    const geometry_msgs__msg__Twist *twist = message;
    k_mutex_lock(&command_mutex, K_FOREVER);
    latest_cmd_linear_x = isfinite(twist->linear.x)
        ? clamp_speed(twist->linear.x, ROBOT_MAX_LINEAR_SPEED_M_S)
        : 0.0f;
    latest_cmd_linear_y = isfinite(twist->linear.y)
        ? clamp_speed(twist->linear.y, ROBOT_MAX_LINEAR_SPEED_M_S)
        : 0.0f;
    latest_cmd_angular_z = isfinite(twist->angular.z)
        ? clamp_speed(twist->angular.z, ROBOT_MAX_ANGULAR_SPEED_RAD_S)
        : 0.0f;
    latest_cmd_timestamp_ms = k_uptime_get();
    command_received = true;
    k_mutex_unlock(&command_mutex);
}

static void set_stamp(builtin_interfaces__msg__Time *stamp)
{
    uint64_t time_ms = rmw_uros_epoch_millis();
    if (time_ms == 0U) {
        time_ms = (uint64_t)k_uptime_get();
    }
    stamp->sec = (int32_t)(time_ms / 1000U);
    stamp->nanosec = (uint32_t)((time_ms % 1000U) * 1000000U);
}

static void set_header(std_msgs__msg__Header *header)
{
    set_stamp(&header->stamp);
}

static rcl_ret_t publish_sensor_data(rcl_publisher_t *imu_pub,
                                     rcl_publisher_t *mag_pub,
                                     rcl_publisher_t *fix_pub,
                                     rcl_publisher_t *pressure_pub,
                                     rcl_publisher_t *temperature_pub,
                                     rcl_publisher_t *wheel_speeds_pub)
{
    ImuData_t imu;
    ImuOrientation_t orientation;
    Imu_getTelemetrySnapshot(&imu, &orientation);

    float half_roll = orientation.froll * 0.5f;
    float half_pitch = orientation.fpitch * 0.5f;
    float half_yaw = orientation.fyaw * 0.5f;
    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);

    set_header(&imu_msg.header);
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    imu_msg.angular_velocity.x = imu.fGyroscope_rad_s[0];
    imu_msg.angular_velocity.y = imu.fGyroscope_rad_s[1];
    imu_msg.angular_velocity.z = imu.fGyroscope_rad_s[2];
    imu_msg.linear_acceleration.x = imu.fAcceleration_m_s2[0];
    imu_msg.linear_acceleration.y = imu.fAcceleration_m_s2[1];
    imu_msg.linear_acceleration.z = imu.fAcceleration_m_s2[2];
    magnetic_field_msg.magnetic_field.x =
        (double)imu.fMagnetometer_uT[0] * 1.0e-6;
    magnetic_field_msg.magnetic_field.y =
        (double)imu.fMagnetometer_uT[1] * 1.0e-6;
    magnetic_field_msg.magnetic_field.z =
        (double)imu.fMagnetometer_uT[2] * 1.0e-6;
    if (rcl_publish(imu_pub, &imu_msg, NULL) != RCL_RET_OK) {
        return RCL_RET_ERROR;
    }

    set_header(&magnetic_field_msg.header);
    if (rcl_publish(mag_pub, &magnetic_field_msg, NULL) != RCL_RET_OK) {
        return RCL_RET_ERROR;
    }

    struct gnss_data gnss_sample;
    if (RTOS_getGnssSample(&gnss_sample)) {
        nav_sat_fix_msg.status.status =
            gnss_sample.info.fix_status == GNSS_FIX_STATUS_NO_FIX ? -1 : 0;
        nav_sat_fix_msg.status.service = 1;
        nav_sat_fix_msg.latitude =
            (double)gnss_sample.nav_data.latitude / 1000000000.0;
        nav_sat_fix_msg.longitude =
            (double)gnss_sample.nav_data.longitude / 1000000000.0;
        nav_sat_fix_msg.altitude =
            (double)gnss_sample.nav_data.altitude / 1000.0;
    } else {
        nav_sat_fix_msg.status.status = -1;
        nav_sat_fix_msg.status.service = 1;
        nav_sat_fix_msg.latitude = 0.0;
        nav_sat_fix_msg.longitude = 0.0;
        nav_sat_fix_msg.altitude = 0.0;
    }
    set_header(&nav_sat_fix_msg.header);
    if (rcl_publish(fix_pub, &nav_sat_fix_msg, NULL) != RCL_RET_OK) {
        return RCL_RET_ERROR;
    }

    float pressure_hpa;
    float temperature_c;
    if (lps22dfSensor_read(&pressure_hpa, &temperature_c)) {
        set_header(&pressure_msg.header);
        pressure_msg.fluid_pressure = (double)pressure_hpa * 100.0;
        if (rcl_publish(pressure_pub, &pressure_msg, NULL) != RCL_RET_OK) {
            return RCL_RET_ERROR;
        }
        set_header(&temperature_msg.header);
        temperature_msg.temperature = temperature_c;
        if (rcl_publish(temperature_pub, &temperature_msg, NULL) != RCL_RET_OK) {
            return RCL_RET_ERROR;
        }
    }

    RTOS_getWheelSpeeds(wheel_speeds_data);
    wheel_speeds_msg.data.data = wheel_speeds_data;
    wheel_speeds_msg.data.size = 4;
    wheel_speeds_msg.data.capacity = 4;
    return rcl_publish(wheel_speeds_pub, &wheel_speeds_msg, NULL);
}

static void destroy_entities(rclc_support_t *support, rcl_node_t *node,
                             rcl_publisher_t pubs[6], bool pub_created[6],
                             rcl_subscription_t *subscription, bool sub_created,
                             rclc_executor_t *executor, bool executor_created,
                             bool node_created, bool support_created)
{
    if (executor_created) {
        if (rclc_executor_fini(executor) != RCL_RET_OK) {
            printk("micro-ROS: executor cleanup failed\r\n");
        }
    }
    if (sub_created) {
        if (rcl_subscription_fini(subscription, node) != RCL_RET_OK) {
            printk("micro-ROS: subscription cleanup failed\r\n");
        }
    }
    for (size_t i = 0; i < 6; i++) {
        if (pub_created[i]) {
            if (rcl_publisher_fini(&pubs[i], node) != RCL_RET_OK) {
                printk("micro-ROS: publisher cleanup failed\r\n");
            }
        }
    }
    if (node_created) {
        if (rcl_node_fini(node) != RCL_RET_OK) {
            printk("micro-ROS: node cleanup failed\r\n");
        }
    }
    if (support_created) {
        if (rclc_support_fini(support) != RCL_RET_OK) {
            printk("micro-ROS: support cleanup failed\r\n");
        }
    }
}

static bool create_entities(rclc_support_t *support, rcl_node_t *node,
                            rcl_publisher_t pubs[6], bool pub_created[6],
                            rcl_subscription_t *subscription, bool *sub_created,
                            rclc_executor_t *executor, bool *executor_created)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    bool support_created = false;
    bool node_created = false;
    rcl_ret_t ret = rclc_support_init(support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        goto fail;
    }
    support_created = true;

    ret = rclc_node_init_default(node, "nucleo_robot", "", support);
    if (ret != RCL_RET_OK) {
        goto fail;
    }
    node_created = true;

#define INIT_PUBLISHER(index, package, type, topic) \
    do { \
        ret = rclc_publisher_init_default(&pubs[index], node, \
            ROSIDL_GET_MSG_TYPE_SUPPORT(package, msg, type), topic); \
        if (ret != RCL_RET_OK) { \
            goto fail; \
        } \
        pub_created[index] = true; \
    } while (0)

    INIT_PUBLISHER(0, sensor_msgs, Imu, "/imu/data");
    INIT_PUBLISHER(1, sensor_msgs, MagneticField, "/imu/mag");
    INIT_PUBLISHER(2, sensor_msgs, NavSatFix, "/gps/fix");
    INIT_PUBLISHER(3, sensor_msgs, FluidPressure, "/environment/pressure");
    INIT_PUBLISHER(4, sensor_msgs, Temperature, "/environment/temperature");
    INIT_PUBLISHER(5, std_msgs, Float32MultiArray, "/wheel_speeds_rpm");
#undef INIT_PUBLISHER

    ret = rclc_subscription_init_default(subscription, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    if (ret != RCL_RET_OK) {
        goto fail;
    }
    *sub_created = true;

    ret = rclc_executor_init(executor, &support->context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        goto fail;
    }
    *executor_created = true;
    ret = rclc_executor_add_subscription(executor, subscription, &cmd_vel_msg,
                                         cmd_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        goto fail;
    }

    (void)rmw_uros_sync_session(1000);
    return true;

fail:
    printk("micro-ROS: entity creation failed (%d)\r\n", (int)ret);
    destroy_entities(support, node, pubs, pub_created, subscription,
                     *sub_created, executor, *executor_created,
                     node_created, support_created);
    *sub_created = false;
    *executor_created = false;
    memset(pub_created, 0, sizeof(bool) * 6);
    return false;
}

static bool initialize_messages(void)
{
    return sensor_msgs__msg__Imu__init(&imu_msg) &&
        sensor_msgs__msg__MagneticField__init(&magnetic_field_msg) &&
        sensor_msgs__msg__NavSatFix__init(&nav_sat_fix_msg) &&
        sensor_msgs__msg__FluidPressure__init(&pressure_msg) &&
        sensor_msgs__msg__Temperature__init(&temperature_msg) &&
        std_msgs__msg__Float32MultiArray__init(&wheel_speeds_msg) &&
        geometry_msgs__msg__Twist__init(&cmd_vel_msg) &&
        rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link") &&
        rosidl_runtime_c__String__assign(&magnetic_field_msg.header.frame_id, "imu_link") &&
        rosidl_runtime_c__String__assign(&nav_sat_fix_msg.header.frame_id, "gps_link") &&
        rosidl_runtime_c__String__assign(&pressure_msg.header.frame_id, "lps22df_link") &&
        rosidl_runtime_c__String__assign(&temperature_msg.header.frame_id, "lps22df_link");
}

void MicroRosNode_run(void)
{
    if (!initialize_messages()) {
        printk("micro-ROS: failed to initialize message buffers\r\n");
        return;
    }
    if (rmw_uros_set_custom_transport(
            MICRO_ROS_FRAMING_REQUIRED,
            (void *)&default_params,
            microros_transport_open,
            microros_transport_close,
            microros_transport_write,
            microros_transport_read) != RMW_RET_OK) {
        printk("micro-ROS: failed to configure USART3 transport\r\n");
        return;
    }

    rcl_publisher_t pubs[6] = {
        rcl_get_zero_initialized_publisher(), rcl_get_zero_initialized_publisher(),
        rcl_get_zero_initialized_publisher(), rcl_get_zero_initialized_publisher(),
        rcl_get_zero_initialized_publisher(), rcl_get_zero_initialized_publisher()
    };
    bool pub_created[6] = {false};
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    bool sub_created = false;
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    bool executor_created = false;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_support_t support;
    bool agent_absent_reported = false;

    while (1) {
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            if (!agent_absent_reported) {
                printk("micro-ROS: agent absent on USART3; retrying\r\n");
                agent_absent_reported = true;
            }
            k_sleep(K_SECONDS(1));
            continue;
        }

        agent_absent_reported = false;
        printk("micro-ROS: ROS 2 Jazzy agent found\r\n");
        if (!create_entities(&support, &node, pubs, pub_created, &subscription,
                             &sub_created, &executor, &executor_created)) {
            node = rcl_get_zero_initialized_node();
            executor = rclc_executor_get_zero_initialized_executor();
            subscription = rcl_get_zero_initialized_subscription();
            support = (rclc_support_t){0};
            k_sleep(K_SECONDS(1));
            continue;
        }

        int64_t last_publish_ms = 0;
        while (1) {
            rcl_ret_t ret = rclc_executor_spin_some(
                &executor, RCL_MS_TO_NS(MICROROS_SPIN_TIMEOUT_MS));
            if (ret != RCL_RET_OK) {
                printk("micro-ROS: executor stopped (%d)\r\n", (int)ret);
                break;
            }

            int64_t now_ms = k_uptime_get();
            if (now_ms - last_publish_ms >= MICROROS_PUBLISH_PERIOD_MS) {
                ret = publish_sensor_data(&pubs[0], &pubs[1], &pubs[2],
                    &pubs[3], &pubs[4], &pubs[5]);
                if (ret != RCL_RET_OK) {
                    printk("micro-ROS: publish failed (%d)\r\n", (int)ret);
                    break;
                }
                last_publish_ms = now_ms;
            }
        }

        destroy_entities(&support, &node, pubs, pub_created, &subscription,
                         sub_created, &executor, executor_created, true, true);
        memset(pub_created, 0, sizeof(pub_created));
        sub_created = false;
        executor_created = false;
        node = rcl_get_zero_initialized_node();
        executor = rclc_executor_get_zero_initialized_executor();
        subscription = rcl_get_zero_initialized_subscription();
        support = (rclc_support_t){0};
        k_sleep(K_SECONDS(1));
    }
}
