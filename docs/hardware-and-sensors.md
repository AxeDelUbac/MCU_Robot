# Hardware and Sensors

## Board and expansion boards

The firmware targets the ST NUCLEO-G0B1RE with these expansion boards:

- **X-NUCLEO-IKS4A1**: LSM6DSO16IS accelerometer/gyroscope, LIS2MDL magnetometer, and LPS22DF pressure/temperature sensor on I2C1.
- **X-NUCLEO-GNSS1A1**: Teseo-LIV3F GNSS receiver with NMEA output on USART1 at 9600 baud.

## GNSS wiring

The overlay assumes the expansion board's factory J3/J4 jumper routing: GNSS USART1 uses Arduino D8/D2 (TX=PA9, RX=PA10). RESET is held inactive on D7 and WAKEUP is held high on D13. SPI1 is disabled to release D13/PA5 for WAKEUP; this pin is also connected to user LED LD4.

For satellite reception, connect the GNSS antenna and place it where it has a clear view of the sky. A `NO_FIX` status can occur while the receiver is acquiring satellites.

## UART assignments

| Interface | Purpose | Settings |
|---|---|---|
| USART2 / ST-Link VCOM | Sensor and GNSS diagnostic output | 115200 baud, 8N1 |
| USART1 | GNSS1A1 NMEA receiver | 9600 baud, 8N1 |
| USART3 | micro-ROS connection to the ROS 2 agent | 115200 baud, 8N1 |

The VCOM reports IMU, LPS22DF, and GNSS data approximately every two seconds. IMU values are printed as acceleration in mm/s², angular rate in µrad/s, magnetic field in nT, and Euler angles in µrad. Pressure is in hPa, temperature in °C, GNSS coordinates in degrees, and altitude in mm.

Zephyr logging is disabled in the current configuration so it does not clutter the diagnostic serial output.
