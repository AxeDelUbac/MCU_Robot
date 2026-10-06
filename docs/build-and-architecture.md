# Build and Task Architecture

## Platform and toolchain

- Board: ST NUCLEO-G0B1RE, target `nucleo_g0b1re/stm32g0b1xx`.
- RTOS: Zephyr.
- Compiler: `arm-zephyr-eabi-gcc` from the Zephyr SDK selected by the build.
- Build system: CMake and Ninja, normally invoked through west.

Use the SDK and workspace configured for this project. Avoid copying machine-specific SDK paths into project files.

## Build

From the application root, configure and build with:

```sh
west build -b nucleo_g0b1re/stm32g0b1xx -d build .
```

Rebuild the existing configuration with:

```sh
west build -d build
```

Flashing writes firmware to the connected board. Do this only when requested and after confirming that the selected target is the intended device:

```sh
west flash -d build
```

The micro-ROS build has additional environment requirements described in the [micro-ROS guide](micro-ros.md).

## Multitask architecture

Application tasks are declared with `K_THREAD_DEFINE` and their thread entry points are implemented in `src/RTOS.c`. Protocol-specific work can be delegated to a module function; for example, `RobotCommunicationTask` calls `MicroRosNode_run` to keep the micro-ROS implementation in its own module.

| Task | Responsibility | Period / behavior |
|---|---|---|
| `MotorRegulationTask` | Reads `/cmd_vel`, computes wheel setpoints and updates control/odometry | 20 Hz |
| `speedMesurementTask` | Samples wheel encoder speeds | 10 Hz |
| `IMUTask` | Updates IMU orientation and prints IMU, pressure and GNSS diagnostics | 2 s |
| `gnss_task` | Receives GNSS samples through a Zephyr message queue | Event-driven |
| `RobotCommunicationTask` | Exchanges robot telemetry and commands with the external system | Client loop |

Shared wheel speed and GNSS data are protected with Zephyr mutexes; GNSS samples are passed through a message queue. Preserve synchronization and safe startup behavior when changing shared state or task timing.

Motor PWM output is not implemented yet. The regulation task still contains a TODO for the hardware movement controller.