#ifndef MOTORGEARBOX_H
#define MOTORGEARBOX_H

#include <Arduino.h>

typedef struct {
    int highPin; // Pin de commande haute
    int lowPin; // Pin de commande basse
    int pwmPin; // Pin PWM pour la vitesse
} motorGearBox_t;

// Initialize an instance (in-place)
void MotorGearBox_init(motorGearBox_t* mg, int highPin, int lowPin, int pwmPin);

// Actions
void MotorGearBox_setMotorDirectionPWM(motorGearBox_t* mg, bool iDirection, int iSpeedInPWM);
void MotorGearBox_setMotorDirectionPercent(motorGearBox_t* mg, bool iDirection, int iSpeedInPercent);
void MotorGearBox_stopMotor(motorGearBox_t* mg);

#endif
