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
        double execute(double input);
        void printDebug();
        void reset();
        void setKeoficients(double kp, double ki, double kd);
        void setTarget(double target);
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