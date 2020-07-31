#ifndef PID_H
#define PID_H

#include "Arduino.h"


class Pid {
    public: 
        Pid(
            double initialKp, 
            double initialKi,
            double initialKd,
            double initialTarget
            ): kp(initialKp), ki(initialKi), kd(initialKd), target(initialTarget) {};
        double pidExecute(double input);
        void pidPrintDebug();
        void pidReset();
        void pidSetKeoficients(double kp, double ki, double kd);
        void pidSetTarget(double target);
        double input = 0;
        double output = 0;
        double prevError = 0;
    private: 
        double kp;
        double ki;
        double kd;
        double target;
        double errorSum = 0;
        uint32_t lastPidSample = 0;
        double lastSampleTime;
};
#endif