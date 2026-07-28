#ifndef RTOS_H
#define RTOS_H

#include <zephyr/kernel.h>

/* Task entry-point prototypes — implementations live in their own modules */
void MotorRegulationTask(void *p1, void *p2, void *p3);
void speedMesurementTask(void *p1, void *p2, void *p3);
void IMUTask(void *p1, void *p2, void *p3);
void gnss_task(void *p1, void *p2, void *p3);
void sensorTask(void *p1, void *p2, void *p3);
void DataCommunicationTask(void *p1, void *p2, void *p3);

#endif /* RTOS_H */
