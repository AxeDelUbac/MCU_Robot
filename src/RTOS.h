#ifndef RTOS_H
#define RTOS_H

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>

/* Zephyr task entry points are implemented in RTOS.c. */
void MotorRegulationTask(void *p1, void *p2, void *p3);
void speedMesurementTask(void *p1, void *p2, void *p3);
void IMUTask(void *p1, void *p2, void *p3);
void gnss_task(void *p1, void *p2, void *p3);
void RobotCommunicationTask(void *p1, void *p2, void *p3);
bool RTOS_getGnssSample(struct gnss_data *sample);
void RTOS_getWheelSpeeds(float speeds[4]);

#endif /* RTOS_H */
