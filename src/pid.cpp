#include "pid.h"

//PID / SPEED

static double errorSum = 0;
static uint32_t lastPidSample = 0;
static double lastSampleTime;

double executePid(double newInputAngle) {
  inputAngle = newInputAngle;
  uint32_t time = millis();
  double error = targetAngle - inputAngle;
  double sampleTime = (time - lastPidSample) / 1000.0; //let's sey it's in sec
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //pidOutput = pidKp*(error) + pidKi*(errorSum)*sampleTime - pidKd*(inputAngle-prevAngle)/sampleTime;
  pidOutput = pidKp*error +  pidKi*errorSum*sampleTime + pidKd*(error-prevError)/sampleTime;

  prevError = error;
  lastPidSample = millis();
  lastSampleTime = sampleTime;
  return pidOutput;
}

void printPidDebug(double inputAngle) {
    static unsigned long lastOutput = 0;
    unsigned long now = millis();
    if (now - lastOutput > 250) {
      lastOutput = now;
      #if defined(DEBUG_PID) 
      Serial.print("PID: "); Serial.print(pidKp); Serial.print(",");Serial.print(pidKi); Serial.print(",");Serial.print(pidKd); Serial.print("; ");
      Serial.print("Angle: "); Serial.println(inputAngle);
      Serial.print("Target Ang.: "); Serial.println(targetAngle);
      Serial.print("PID output: "); Serial.print(pidOutput);
      Serial.print(" speed "); Serial.print(speed);
      Serial.print(" prev error: "); Serial.print(prevError);
      Serial.print(" error sum: "); Serial.print(errorSum);
      Serial.print(" last time: "); Serial.print(lastSampleTime);
      Serial.println();
      Serial.print("Motors on: "); Serial.print(RemoteXY.motorsOn);
      Serial.print(" connection flag: "); Serial.print(RemoteXY.connect_flag);
      Serial.println();
      printSpeedInfoToSerial();
      motorPrintDebug();
      #endif
    }
}

void pidReset() {
    errorSum = 0;
    prevError = 0;
    pidOutput = 0;
}

void pidSetKeoficients(double kp, double ki, double kd) {
    pidKp = kp;
    pidKi = ki;
    pidKd = kd;
}