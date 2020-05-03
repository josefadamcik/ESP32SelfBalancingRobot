#ifndef IMUHEADER_H
#define IMUHEADER_H

#include "FunctionalInterrupt.h"

void setupMPU6050(const uint8_t interruptPin, const char * preferencesNamespace, std::function<void(void)> dmpReadyIsr);
void calibrateMPU(const char * preferencesNamespace);
bool isMPUReady();
bool getYPR(float *data);

#endif