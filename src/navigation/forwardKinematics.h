#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

typedef struct {
    int vx;      // Vitesse longitudinale (-100 à +100)
    int vy;      // Vitesse latérale (-100 à +100)
    int omega;   // Vitesse angulaire (-100 à +100)
    float motorSpeeds[4];           // Vitesses des roues [FL, FR, RL, RR]
    float normalisedMotorSpeeds[4]; // Vitesses normalisées [FL, FR, RL, RR]
    float geometricFactor;          // Facteur géométrique (L + W)
} forwardKinematics_t;

void ForwardKinematics_computeWheelVelocities(forwardKinematics_t* fk, float normalisedSpeeds[4]);
void ForwardKinematics_normalizeMotorsSpeeds(forwardKinematics_t* fk, float maxValue);
void ForwardKinematics_debug(forwardKinematics_t* fk);

#endif /* FORWARDKINEMATICS_H */
