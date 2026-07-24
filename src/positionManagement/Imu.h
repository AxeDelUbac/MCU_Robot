#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void Imu_init(void);
void Imu_getAccelerometer(void);
void Imu_getGyroscope(void);
void Imu_getMagnetometer(void);
void Imu_SerialDebug(void);
void Imu_updateOrientation(float dt);

void Imu_getAcceleration_m_s2(void);
void Imu_getAngularRate_deg_s(float out_deg_s[3]);
void Imu_getAngularRate_rad_s(void);

typedef struct {
    int32_t i32Acceleration[3];    /* integer part of accel (m/s²) */
    int32_t i32Gyroscope[3];       /* integer part of gyro  (rad/s) */
    int32_t i32Magnetometer[3];    /* integer part of magn  (µT)   */
    float   fAcceleration_m_s2[3];
    float   fGyroscope_rad_s[3];
    float   fMagnetometer_uT[3];
} ImuData_t;

typedef struct {
    float froll;
    float fpitch;
    float fyaw;
} ImuOrientation_t;

extern ImuData_t       imuData;
extern ImuOrientation_t imuOrientation;

#endif /* IMU_H */
