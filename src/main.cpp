#include "Arduino.h"
// #define CUSTOM_SETTINGS
// #define INCLUDE_TERMINAL_MODULE
// #define INCLUDE_MOTORCONTROL_MODULE
// #include <DabbleESP32.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "ESP32MotorControl.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU6050 mpu;
// #define OUTPUT_TEAPOT
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

#define MOTOR_A1 32
#define MOTOR_A2 33
#define MOTOR_B1 25
#define MOTOR_B2 26

const int pwmFreq = 10000;
const int pwmMotorAChannel = 0;
const int pwmMotorBChannel = 1;
const int pwmResolution = 10;
const int pwmMinDuty = 400;
const int pwmMaxDuty = 1023;
ESP32MotorControl MotorControl = ESP32MotorControl();

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // digitalWrite(MOTOR_A1, LOW);
  // digitalWrite(MOTOR_A2, LOW);
  // digitalWrite(MOTOR_B1, LOW);
  // digitalWrite(MOTOR_B2, LOW);
  // ledcSetup(pwmMotorAChannel, pwmFreq, pwmResolution);
  // ledcSetup(pwmMotorBChannel, pwmFreq, pwmResolution);
  MotorControl.attachMotors(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
  Serial.begin(115200);
}

int counter = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  Serial.println("loop");
  for (int i = 0; i <= 100; i++) {
    MotorControl.motorForward(0, i);
    MotorControl.motorForward(1, i);
    delay(100);
  }
  // ledcAttachPin(MOTOR_A1, pwmMotorAChannel);
  // ledcAttachPin(MOTOR_B1, pwmMotorBChannel);
  // digitalWrite(MOTOR_A2, LOW); //Fast decay
  // digitalWrite(MOTOR_B2, LOW);
  // for (int i = pwmMinDuty; i <= pwmMaxDuty; i++) {
  //   Serial.println(i);
  //   ledcWrite(pwmMotorAChannel, i);
  //   ledcWrite(pwmMotorBChannel, i);
  //   delay(10);
  // }
  // delay(1000);
  // for (int i = pwmMaxDuty; i >= pwmMinDuty; i--) {
  //   Serial.println(i);
  //   ledcWrite(pwmMotorAChannel, i);
  //   ledcWrite(pwmMotorBChannel, i);
  //   delay(10);
  // }
  // ledcDetachPin(MOTOR_A1);
  // ledcDetachPin(MOTOR_B1);
  // ledcAttachPin(MOTOR_A2, pwmMotorAChannel);
  // ledcAttachPin(MOTOR_B2, pwmMotorBChannel);
  // digitalWrite(MOTOR_A1, LOW); //Fast decay
  // digitalWrite(MOTOR_B1, LOW);
  // delay(100);
  // for (int i = pwmMinDuty; i <= pwmMaxDuty; i++) {
  //   Serial.println(i);
  //   ledcWrite(pwmMotorAChannel, i);
  //   ledcWrite(pwmMotorBChannel, i);
  //   delay(10);
  // }
  // delay(1000);
  // for (int i = pwmMaxDuty; i >= pwmMinDuty; i--) {
  //   Serial.println(i);
  //   ledcWrite(pwmMotorAChannel, i);
  //   ledcWrite(pwmMotorBChannel, i);
  //   delay(10);
  // }
  // delay(100);
  // ledcDetachPin(MOTOR_A2);
  // ledcDetachPin(MOTOR_B2);

}
