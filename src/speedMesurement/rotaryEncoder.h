#ifndef ENCODEUR_H
#define ENCODEUR_H

#include "encoderParameter.h"

typedef struct {
    float fAngularSpeedInRpm;
    float fLinearSpeedInKmH;
    int   iMotorDirection;
} rotaryEncoder_t;

float rotaryEncoder_getSpeedRpm(rotaryEncoder_t *enc, int ipulse, float fPeriodMs);
float rotaryEncoder_getSpeedKmH(rotaryEncoder_t *enc, int ipulse, float fPeriodMs);
void  rotaryEncoder_setDirection(rotaryEncoder_t *enc, int iDirection);

#endif /* ENCODEUR_H */
