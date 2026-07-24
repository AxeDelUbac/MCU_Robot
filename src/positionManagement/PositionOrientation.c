#include "PositionOrientation.h"

void PositionOrientation_init(void)
{
    Imu_init();
}

void PositionOrientation_update(float dt)
{
    Imu_updateOrientation(dt);
}

void PositionOrientation_getEulerAngles(float *outRoll, float *outPitch, float *outYaw)
{
    *outRoll  = imuOrientation.froll;
    *outPitch = imuOrientation.fpitch;
    *outYaw   = imuOrientation.fyaw;
}
