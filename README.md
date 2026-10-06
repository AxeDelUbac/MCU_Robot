# MCU Robot Firmware

Firmware embarqué en C pour la carte ST NUCLEO-G0B1RE. Le projet utilise Zephyr RTOS, CMake/west et le compilateur GCC fourni avec le Zephyr SDK. Il lit les capteurs des extensions ST, affiche leur état sur l’UART et intègre un client micro-ROS.

## Documentation

- [Index de la documentation](docs/README.md)
- [Build, carte et architecture multitâche](docs/build-and-architecture.md)
- [Capteurs et ports UART](docs/hardware-and-sensors.md)
- [micro-ROS Jazzy : câblage, topics et agent](docs/micro-ros.md)