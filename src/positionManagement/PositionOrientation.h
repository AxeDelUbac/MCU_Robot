#ifndef POSITIONORIENTATION_H
#define POSITIONORIENTATION_H

#include "Imu.h"

void PositionOrientation_init(void);
void PositionOrientation_update(float dt);
void PositionOrientation_getEulerAngles(float *outRoll, float *outPitch, float *outYaw);

#endif /* POSITIONORIENTATION_H */
