#ifndef CLOSEDLOOPCONTROL_H
#define CLOSEDLOOPCONTROL_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float lastError;
    float integral;
    float prevDerivativeFiltered;

    float fOutputPID;
    float fErrorPID;
} ClosedLoopControl;

// Initialise la structure ClosedLoopControl
void ClosedLoopControl_init(ClosedLoopControl* ctl, float Kp, float Ki, float Kd);

// Met à jour le PID et retourne la sortie (PWM attendu)
float ClosedLoopControl_updatePIDControl(ClosedLoopControl* ctl, float setpoint, float measuredSpeed);

// Change les gains
void ClosedLoopControl_setTunings(ClosedLoopControl* ctl, float Kp, float Ki, float Kd);

// Getters
float ClosedLoopControl_getOutputPID(ClosedLoopControl* ctl);
float ClosedLoopControl_getErrorPID(ClosedLoopControl* ctl);

// Saturation helper
float ClosedLoopControl_thresholdPID(ClosedLoopControl* ctl, float pwmOutput);

#endif // CLOSEDLOOPCONTROL_H