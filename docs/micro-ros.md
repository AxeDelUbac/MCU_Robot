# micro-ROS Jazzy

The firmware uses the `micro_ros_zephyr_module` submodule to build a micro-ROS client for ROS 2 Jazzy. The client communicates with a micro-ROS agent on the SBC over USART3.

## UART connection

Connect the NUCLEO-G0B1RE USART3 pins to a 3.3 V UART on the SBC:

- PC10 / TX -> SBC RX
- PC11 / RX <- SBC TX
- Connect the grounds.
- Configure both ends for 115200 baud, 8N1.

Do not connect UART pins using incompatible voltage levels. USART2/ST-Link VCOM remains the separate sensor diagnostic output.

## Start the agent

On the SBC, use the device path for the UART connected to PC10/PC11:

```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Replace `/dev/ttyUSB0` with the actual serial device. The firmware retries the connection when the agent is absent.

## Topics

Published approximately every two seconds:

| Topic | Type | Data |
|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | Acceleration, angular velocity, and integrated orientation |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetic field in teslas |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GNSS fix and position |
| `/environment/pressure` | `sensor_msgs/msg/FluidPressure` | Pressure in pascals |
| `/environment/temperature` | `sensor_msgs/msg/Temperature` | Temperature in °C |
| `/wheel_speeds_rpm` | `std_msgs/msg/Float32MultiArray` | Wheel speeds in `[FL, FR, RL, RR]` order |

The firmware subscribes to `geometry_msgs/msg/Twist` on `/cmd_vel`. It clamps `linear.x` and `linear.y` to ±1 m/s and `angular.z` to ±1 rad/s; these initial assumed limits map to ±100 in the existing kinematics. If no fresh command arrives for 500 ms, the setpoints return to zero.

## Current limitations

- The agent connection and ROS topics must be verified with the SBC connected; successful firmware compilation alone does not verify ROS communication.
- The motion setpoints reach the existing kinematics and control calculation, but motor PWM output is not implemented. `/cmd_vel` therefore does not yet drive the motors.
- The command limits are starting assumptions and must be validated against the robot.

## Build environment

Build the firmware with the Zephyr workspace and SDK configured for this project. The micro-ROS module also requires its build environment to avoid inherited ROS/Python environment settings that conflict with its pinned dependencies. Use the established project build setup rather than adding workstation-specific paths to source files.
