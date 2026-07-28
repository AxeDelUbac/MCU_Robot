#include "forwardKinematics.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

void ForwardKinematics_computeWheelVelocities(forwardKinematics_t* fk, float normalisedSpeeds[4])
{
    if (fk->vx >  100) fk->vx =  100;
    if (fk->vx < -100) fk->vx = -100;
    if (fk->vy >  100) fk->vy =  100;
    if (fk->vy < -100) fk->vy = -100;
    if (fk->omega >  100) fk->omega =  100;
    if (fk->omega < -100) fk->omega = -100;

    fk->motorSpeeds[0] = fk->vx - fk->vy - (fk->geometricFactor * fk->omega); /* FL */
    fk->motorSpeeds[1] = fk->vx + fk->vy + (fk->geometricFactor * fk->omega); /* FR */
    fk->motorSpeeds[2] = fk->vx + fk->vy - (fk->geometricFactor * fk->omega); /* RL */
    fk->motorSpeeds[3] = fk->vx - fk->vy + (fk->geometricFactor * fk->omega); /* RR */

    ForwardKinematics_normalizeMotorsSpeeds(fk, 100.0f);

    for (int i = 0; i < 4; i++) {
        normalisedSpeeds[i] = fk->normalisedMotorSpeeds[i];
    }
}

void ForwardKinematics_normalizeMotorsSpeeds(forwardKinematics_t* fk, float maxValue)
{
    float maxSpeed = 0.0f;
    for (int i = 0; i < 4; i++) {
        float abs = fk->motorSpeeds[i] < 0 ? -fk->motorSpeeds[i] : fk->motorSpeeds[i];
        if (abs > maxSpeed) maxSpeed = abs;
    }

    if (maxSpeed > maxValue) {
        float scale = maxValue / maxSpeed;
        for (int i = 0; i < 4; i++) fk->normalisedMotorSpeeds[i] = fk->motorSpeeds[i] * scale;
    } else {
        for (int i = 0; i < 4; i++) fk->normalisedMotorSpeeds[i] = fk->motorSpeeds[i];
    }
}

void ForwardKinematics_debug(forwardKinematics_t* fk)
{
    LOG_INF("FK  vx=%d vy=%d omega=%d", fk->vx, fk->vy, fk->omega);
    LOG_INF("FK  motors FL=%d FR=%d RL=%d RR=%d",
        (int)fk->motorSpeeds[0], (int)fk->motorSpeeds[1],
        (int)fk->motorSpeeds[2], (int)fk->motorSpeeds[3]);
    LOG_INF("FK  norm   FL=%d FR=%d RL=%d RR=%d",
        (int)fk->normalisedMotorSpeeds[0], (int)fk->normalisedMotorSpeeds[1],
        (int)fk->normalisedMotorSpeeds[2], (int)fk->normalisedMotorSpeeds[3]);
}
