#include "rotaryEncoder.h"
#include <math.h>

float rotaryEncoder_getSpeedRpm(rotaryEncoder_t *enc, int ipulse, float fPeriodMs)
{
    enc->fAngularSpeedInRpm =
        (ipulse * 60000.0f) / (PULSES_PER_ROTATION * MOTOR_REDUCTION_RATIO * fPeriodMs);
    return enc->fAngularSpeedInRpm;
}

float rotaryEncoder_getSpeedKmH(rotaryEncoder_t *enc, int ipulse, float fPeriodMs)
{
    float rpm = rotaryEncoder_getSpeedRpm(enc, ipulse, fPeriodMs);
    enc->fLinearSpeedInKmH = rpm * 3.14159265f * WHEEL_DIAMETER_M / 60.0f * 3.6f;
    return enc->fLinearSpeedInKmH;
}

void rotaryEncoder_setDirection(rotaryEncoder_t *enc, int iDirection)
{
    enc->iMotorDirection = iDirection;
}
