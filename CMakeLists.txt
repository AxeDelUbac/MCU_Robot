cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)

target_sources(app PRIVATE
  src/main.c
  src/RTOS.c
  src/positionManagement/Imu.c
  src/positionManagement/PositionOrientation.c
  src/sensorManagement/sensorManagement.c
  src/sensorManagement/environmentalSensor.c
  src/sensorManagement/lightSensor.c
  src/speedMesurement/GlobalSpeed.c
  src/speedMesurement/rotaryEncoder.c
  src/speedControlSystem/GlobalControl.c
  src/speedControlSystem/closedLoopControl.c
  src/navigation/forwardKinematics.c
  src/navigation/mecanumOdometrie.c
  src/DataCommunication/SerialDataTransmitter.c
  src/DataCommunication/SerialDataReceiver.c
)
