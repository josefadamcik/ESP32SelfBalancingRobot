#ifndef IMU_H
#define IMU_H 

#include "Arduino.h"

void setupMPU6050(const uint8_t interruptPin, const char * preferencesNamespace, void (*dmpReadyIsr)(void));
void calibrateMPU(const char * preferencesNamespace);
bool isMPUReady();
bool getYPR(float *data);

#endif