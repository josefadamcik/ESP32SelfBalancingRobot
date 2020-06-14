#include "pid.h"

double pidExecute(double newInputAngle) {
  Pid.input = newInputAngle;
  uint32_t time = millis();
  double error = Pid.target - Pid.input;
  double sampleTime = (time - Pid.lastPidSample) / 1000.0; //let's sey it's in sec
  Pid.errorSum = Pid.errorSum + error;
  Pid.errorSum = constrain(Pid.errorSum, -300, 300);
  //output = kp*(error) + ki*(errorSum)*sampleTime - kd*(input-prevAngle)/sampleTime;
  Pid.output = Pid.kp*error + Pid.ki*Pid.errorSum*sampleTime + Pid.kd*(error-Pid.prevError)/sampleTime;

  Pid.prevError = error;
  Pid.lastPidSample = millis();
  Pid.lastSampleTime = sampleTime;
  return Pid.output;
}

void pidPrintDebug() {
  Serial.print("PID: KP "); Serial.print(Pid.kp); 
  Serial.print(", KI ");Serial.print(Pid.ki); 
  Serial.print(", KD ");Serial.print(Pid.kd); 
  Serial.print("; ");
  Serial.print("input: "); Serial.println(Pid.input);
  Serial.print(" target: "); Serial.println(Pid.target);
  Serial.print(" output: "); Serial.print(Pid.output);
  Serial.print(" prev error: "); Serial.print(Pid.prevError);
  Serial.print(" error sum: "); Serial.print(Pid.errorSum);
  Serial.print(" last time: "); Serial.println(Pid.lastSampleTime);
}

void pidReset() {
    Pid.errorSum = 0;
    Pid.prevError = 0;
    Pid.output = 0;
}

void pidSetKeoficients(double kp, double ki, double kd) {
    Pid.kp = kp;
    Pid.ki = ki;
    Pid.kd = kd;
}
void pidSetTarget(double target) {
  Pid.target = target;
}