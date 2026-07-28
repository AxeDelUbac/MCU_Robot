#include "mecanumOdometrie.h"
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

#define PI_F          3.14159265f
#define RPM_TO_RAD_S  (2.0f * PI_F / 60.0f)

void MecanumOdometry_init(MecanumOdometry_t* odometry, float wheelRadius,
                          float wheelBase, float trackWidth)
{
    odometry->wheelRadius = wheelRadius;
    odometry->wheelBase   = wheelBase;
    odometry->trackWidth  = trackWidth;
    MecanumOdometry_reset(odometry);
    odometry->initialized = true;
}

void MecanumOdometry_reset(MecanumOdometry_t* odometry)
{
    odometry->pose.x     = 0.0f;
    odometry->pose.y     = 0.0f;
    odometry->pose.theta = 0.0f;

    odometry->velocity.vx    = 0.0f;
    odometry->velocity.vy    = 0.0f;
    odometry->velocity.omega = 0.0f;

    odometry->lastUpdateTime = k_uptime_get();
}

void MecanumOdometry_setPose(MecanumOdometry_t* odometry, float x, float y, float theta)
{
    odometry->pose.x     = x;
    odometry->pose.y     = y;
    odometry->pose.theta = MecanumOdometry_normalizeAngle(theta);
}

void MecanumOdometry_updateWheelSpeeds(MecanumOdometry_t* odometry, float wheelSpeed[4])
{
    float wheelVelocities[4];
    MecanumOdometry_rpmArrayToMetersPerSecond(odometry, wheelSpeed, wheelVelocities);

    int64_t now = k_uptime_get();
    float deltaTime = (float)(now - odometry->lastUpdateTime) / 1000.0f;

    MecanumOdometry_updatePose(odometry, wheelVelocities, deltaTime);
    odometry->lastUpdateTime = now;
}

void MecanumOdometry_updatePose(MecanumOdometry_t* odometry,
                                float wheelVelocities[4], float deltaTime)
{
    MecanumOdometry_updateKinematics(odometry, wheelVelocities);

    float cos_theta = cosf(odometry->pose.theta);
    float sin_theta = sinf(odometry->pose.theta);

    float vx_world = odometry->velocity.vx * cos_theta - odometry->velocity.vy * sin_theta;
    float vy_world = odometry->velocity.vx * sin_theta + odometry->velocity.vy * cos_theta;

    odometry->pose.x     += vx_world * deltaTime;
    odometry->pose.y     += vy_world * deltaTime;
    odometry->pose.theta += odometry->velocity.omega * deltaTime;
    odometry->pose.theta  = MecanumOdometry_normalizeAngle(odometry->pose.theta);
}

void MecanumOdometry_updateKinematics(MecanumOdometry_t* odometry, float wheelVel[4])
{
    float lx = odometry->wheelBase  / 2.0f;
    float ly = odometry->trackWidth / 2.0f;

    float scale_linear  = odometry->wheelRadius / 4.0f;
    float scale_angular = odometry->wheelRadius / (4.0f * (lx + ly));

    odometry->velocity.vx    = scale_linear  * ( wheelVel[0] + wheelVel[1] + wheelVel[2] + wheelVel[3]);
    odometry->velocity.vy    = scale_linear  * (-wheelVel[0] + wheelVel[1] + wheelVel[2] - wheelVel[3]);
    odometry->velocity.omega = scale_angular * (-wheelVel[0] + wheelVel[1] - wheelVel[2] + wheelVel[3]);
}

float MecanumOdometry_getDistanceTraveled(const MecanumOdometry_t* odometry,
                                          float startX, float startY)
{
    float dx = odometry->pose.x - startX;
    float dy = odometry->pose.y - startY;
    return sqrtf(dx * dx + dy * dy);
}

float MecanumOdometry_normalizeAngle(float angle)
{
    if      (angle >  PI_F) angle -= 2.0f * PI_F;
    else if (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

void MecanumOdometry_rpmArrayToMetersPerSecond(MecanumOdometry_t* odometry,
                                               float rpmArray[4], float velocityArray[4])
{
    float fRpmToMs = RPM_TO_RAD_S * odometry->wheelRadius;
    for (int i = 0; i < 4; i++) {
        velocityArray[i] = rpmArray[i] * fRpmToMs;
    }
}

void MecanumOdometry_debug(const MecanumOdometry_t* odometry)
{
    if (!odometry || !odometry->initialized) {
        LOG_ERR("Odometry not initialized");
        return;
    }
    LOG_INF("ODO  x=%d mm  y=%d mm  theta=%d mrad",
        (int)(odometry->pose.x     * 1000),
        (int)(odometry->pose.y     * 1000),
        (int)(odometry->pose.theta * 1000));
    LOG_INF("ODO  vx=%d mm/s  vy=%d mm/s  omega=%d mrad/s",
        (int)(odometry->velocity.vx    * 1000),
        (int)(odometry->velocity.vy    * 1000),
        (int)(odometry->velocity.omega * 1000));
}
