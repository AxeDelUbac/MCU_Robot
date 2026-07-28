#ifndef GLOBALCONTROL_H
#define GLOBALCONTROL_H

#include "closedLoopControl.h"

typedef struct {
    ClosedLoopControl closedLoopFrontLeft;
    ClosedLoopControl closedLoopFrontRight;
    ClosedLoopControl closedLoopRearLeft;
    ClosedLoopControl closedLoopRearRight;
    float ftabSetpointKmh[4];
} GlobalControl;

void GlobalControl_init(GlobalControl* gc);
void GlobalControl_UpdateSetpoint(GlobalControl* gc, float fSetpointKmh[4], float fMeasuredSpeedKmh[4], float fOutputKmh[4]);
void GlobalControl_SerialDebug(GlobalControl* gc);

#endif