#include "GlobalControl.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

void GlobalControl_init(GlobalControl* gc) {
    ClosedLoopControl_init(&gc->closedLoopFrontLeft,  2.4f, 1.13f, 0.00f);
    ClosedLoopControl_init(&gc->closedLoopFrontRight, 2.4f, 1.13f, 0.05f);
    ClosedLoopControl_init(&gc->closedLoopRearLeft,   2.4f, 1.13f, 0.05f);
    ClosedLoopControl_init(&gc->closedLoopRearRight,  2.4f, 1.13f, 0.05f);
    for (int i = 0; i < 4; i++) gc->ftabSetpointKmh[i] = 0.0f;
}

void GlobalControl_UpdateSetpoint(GlobalControl* gc, float fSetpointKmh[4],
                                  float fMeasuredSpeedKmh[4], float fOutputKmh[4]) {
    fOutputKmh[0] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontLeft,  fSetpointKmh[0], fMeasuredSpeedKmh[0]);
    fOutputKmh[1] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontRight, fSetpointKmh[1], fMeasuredSpeedKmh[1]);
    fOutputKmh[2] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearLeft,   fSetpointKmh[2], fMeasuredSpeedKmh[2]);
    fOutputKmh[3] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearRight,  fSetpointKmh[3], fMeasuredSpeedKmh[3]);
}

void GlobalControl_SerialDebug(GlobalControl* gc) {
    LOG_INF("PID FL  err=%d out=%d",
        (int)(ClosedLoopControl_getErrorPID(&gc->closedLoopFrontLeft)  * 1000),
        (int)(ClosedLoopControl_getOutputPID(&gc->closedLoopFrontLeft) * 1000));
    LOG_INF("PID FR  err=%d out=%d",
        (int)(ClosedLoopControl_getErrorPID(&gc->closedLoopFrontRight)  * 1000),
        (int)(ClosedLoopControl_getOutputPID(&gc->closedLoopFrontRight) * 1000));
    LOG_INF("PID RL  err=%d out=%d",
        (int)(ClosedLoopControl_getErrorPID(&gc->closedLoopRearLeft)  * 1000),
        (int)(ClosedLoopControl_getOutputPID(&gc->closedLoopRearLeft) * 1000));
    LOG_INF("PID RR  err=%d out=%d",
        (int)(ClosedLoopControl_getErrorPID(&gc->closedLoopRearRight)  * 1000),
        (int)(ClosedLoopControl_getOutputPID(&gc->closedLoopRearRight) * 1000));
}
