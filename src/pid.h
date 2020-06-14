#ifndef PID_H
#define PID_H

#include "Arduino.h"

double const initialPidKp=8, initialPidKi = 18, initialPikKd = 0.2;
double const initialTargetAngle = 2.2;
typedef struct {
    double target = initialTargetAngle;
    double kp=initialPidKp;
    double ki=initialPidKi;
    double kd=initialPikKd;
    double input = 0;
    double output = 0;
    double prevError = 0;
    double errorSum = 0;
    uint32_t lastPidSample = 0;
    double lastSampleTime;
} pid_state;

static pid_state Pid;

double pidExecute(double input);
void pidPrintDebug();
void pidReset();
void pidSetKeoficients(double kp, double ki, double kd);
void pidSetTarget(double target);

#endif