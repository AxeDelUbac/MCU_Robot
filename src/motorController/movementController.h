#ifndef MOVEMENTCONTROLLER_H
#define MOVEMENTCONTROLLER_H

#include <Arduino.h>
#include "GPIO.h"
#include "motorGearBox.h"

// C-style API: struct + functions
typedef struct {
    motorGearBox_t orightBackMotor;
    motorGearBox_t oleftBackMotor;
    motorGearBox_t orightFrontMotor;
    motorGearBox_t oleftFrontMotor;
    float fTabSpeedInPwm[4];
    float fTabSpeedInPercent[4];
    bool bTabDirection[4];
} movementController_t;

// Lifecycle
void MovementController_init(movementController_t* mc);

// Actions
void MovementController_setMovement(movementController_t* mc, float fMotorSpeeds[4]);

void MovementController_movementFront(movementController_t* mc);
void MovementController_movementBack(movementController_t* mc);
void MovementController_movementRight(movementController_t* mc);
void MovementController_movementLeft(movementController_t* mc);

void MovementController_movementFrontLeft(movementController_t* mc);
void MovementController_movementFrontRight(movementController_t* mc);

void MovementController_movementstop(movementController_t* mc);

void MovementController_setMotorSpeedInPWM(movementController_t* mc, float fTabSpeed[4]);

#endif