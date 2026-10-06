cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/micro_ros_zephyr_module/modules/libmicroros
)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)

zephyr_include_directories(
  ${CMAKE_BINARY_DIR}/zephyr/include/generated/zephyr
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(app PRIVATE __STDC_WANT_LIB_EXT1__=0)

target_sources(app PRIVATE
  src/main.c
  src/RTOS.c
  src/positionManagement/Imu.c
  src/positionManagement/PositionOrientation.c
  src/DataCommunication/MicroRosNode.c
  src/DataCommunication/micro_ros_atomic64.c
  src/sensorManagement/lps22dfSensor.c
  src/speedMesurement/GlobalSpeed.c
  src/speedMesurement/rotaryEncoder.c
  src/speedControlSystem/GlobalControl.c
  src/speedControlSystem/closedLoopControl.c
  src/navigation/forwardKinematics.c
  src/navigation/mecanumOdometrie.c
)
