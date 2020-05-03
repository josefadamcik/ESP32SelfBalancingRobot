#ifndef IMUHEADER_H
#define IMUHEADER_H

#include "FunctionalInterrupt.h"
#include "Arduino.h"
#include "Preferences.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

void setupMPU6050(const uint8_t interruptPin, const char * preferencesNamespace, std::function<void(void)> dmpReadyIsr);
void calibrateMPU(const char * preferencesNamespace);
boolean isMPUReady();
boolean getYPR(float *data);

#endif