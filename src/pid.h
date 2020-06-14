#ifndef PID_H
#define PID_H

#include "Arduino.h"

double const initialPidKp=8, initialPidKi = 18, initialPikKd = 0.2;
double const initialTargetAngle = 2.2;

double targetAngle = initialTargetAngle;
double inputAngle = 0;
double prevError = 0;
static double pidOutput = 0;
static double pidKp=initialPidKp, pidKi=initialPidKi, pidKd=initialPikKd;


double executePid(double inputAngle);
void printPidDebug(double inputAngle);
void pidReset();
void pidSetKeoficients(double kp, double ki, double kd);

#endif