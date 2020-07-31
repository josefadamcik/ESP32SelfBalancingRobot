#include "pid.h"

double Pid::execute(double newInputAngle) {
    input = newInputAngle;
    uint32_t time = millis();
    double error = target - input;
    double sampleTime =
        (time - lastPidSample) / 1000.0;  // let's sey it's in sec
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);
    output = kp * error + ki * errorSum * sampleTime +
             kd * (error - prevError) / sampleTime;
    prevError = error;
    lastPidSample = millis();
    lastSampleTime = sampleTime;
    return output;
}

void Pid::printDebug() {
    Serial.print("PID: KP ");
    Serial.print(kp);
    Serial.print(", KI ");
    Serial.print(ki);
    Serial.print(", KD ");
    Serial.print(kd);
    Serial.print("; ");
    Serial.print("input: ");
    Serial.println(input);
    Serial.print(" target: ");
    Serial.println(target);
    Serial.print(" output: ");
    Serial.print(output);
    Serial.print(" prev error: ");
    Serial.print(prevError);
    Serial.print(" error sum: ");
    Serial.print(errorSum);
    Serial.print(" last time: ");
    Serial.println(lastSampleTime);
}

void Pid::reset() {
    errorSum = 0;
    prevError = 0;
    output = 0;
}

void Pid::setKeoficients(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
void Pid::setTarget(double target) { this->target = target; }