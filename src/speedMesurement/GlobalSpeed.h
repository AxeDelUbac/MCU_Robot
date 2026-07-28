#ifndef GLOBALSPEED_H
#define GLOBALSPEED_H

#include "rotaryEncoder.h"

typedef struct {
    rotaryEncoder_t rotaryEncoderFrontLeft;
    rotaryEncoder_t rotaryEncoderFrontRight;
    rotaryEncoder_t rotaryEncoderRearLeft;
    rotaryEncoder_t rotaryEncoderRearRight;
    float fWheelSpeed[4];   /* RPM par roue : [0]=FL [1]=FR [2]=RL [3]=RR */
    float fMeanRpm;
} GlobalSpeed_t;

extern GlobalSpeed_t tGlobalSpeed;

void  GlobalSpeed_init(void);
float GlobalSpeed_getMeanSpeedInRPM(float *fSpeedEncodeRPM, float fPeriodMs);
void  GlobalSpeed_debug(void);

#endif /* GLOBALSPEED_H */
