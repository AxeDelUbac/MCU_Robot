#ifndef MECANUM_ODOMETRIE_H
#define MECANUM_ODOMETRIE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float x;      // Position X (m)
    float y;      // Position Y (m)
    float theta;  // Orientation (rad)
} RobotPose_t;

typedef struct {
    float vx;    // Vitesse linéaire X (m/s)
    float vy;    // Vitesse linéaire Y (m/s)
    float omega; // Vitesse angulaire (rad/s)
} RobotVelocity_t;

typedef struct {
    RobotPose_t     pose;
    RobotVelocity_t velocity;
    float           wheelRadius;
    float           wheelBase;
    float           trackWidth;
    int64_t         lastUpdateTime; // ms (k_uptime_get)
    bool            initialized;
} MecanumOdometry_t;

void  MecanumOdometry_init(MecanumOdometry_t* odometry, float wheelRadius, float wheelBase, float trackWidth);
void  MecanumOdometry_reset(MecanumOdometry_t* odometry);
void  MecanumOdometry_setPose(MecanumOdometry_t* odometry, float x, float y, float theta);
void  MecanumOdometry_updateWheelSpeeds(MecanumOdometry_t* odometry, float wheelSpeed[4]);
void  MecanumOdometry_updatePose(MecanumOdometry_t* odometry, float wheelVelocities[4], float deltaTime);
void  MecanumOdometry_updateKinematics(MecanumOdometry_t* odometry, float wheelVelocities[4]);
float MecanumOdometry_getDistanceTraveled(const MecanumOdometry_t* odometry, float startX, float startY);
float MecanumOdometry_normalizeAngle(float angle);
void  MecanumOdometry_rpmArrayToMetersPerSecond(MecanumOdometry_t* odometry, float rpmArray[4], float velocityArray[4]);
void  MecanumOdometry_debug(const MecanumOdometry_t* odometry);

#endif /* MECANUM_ODOMETRIE_H */
