
/*
   Terminal Module is like a chat box. It allows you to send and receive commands between your
   board and smartphone.

   You can reduce the size of library compiled by enabling only those modules that you
   want to use. For this first define CUSTOM_SETTINGS followed by defining
   INCLUDE_modulename.

   Explore more on: https://thestempedia.com/docs/dabble/terminal-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#define INCLUDE_MOTORCONTROL_MODULE
#include <DabbleESP32.h>
// #include <MPU6050_tockn.h>
// #include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// MPU6050 mpu6050(Wire);
MPU6050 accelgyro;
String Serialdata;
bool dataflag = 0;
long timer = 0;
const uint8_t pwmMotor1 =  14;
const uint8_t dir1Motor1 = 12;
const uint8_t dir2Motor1 = 13;
// const int freq = 30000; //30kHz
const int freq = 3000; //3kHz
const int pwmChannel = 0;
const int resolution = 8;
const uint8_t sdaPin = 21;
const uint8_t sclPin = 22;

int16_t accY, accZ;
float accAngleX;
float accAngleY;

void setup() {
  Wire.begin(sdaPin, sclPin, 400000); 
  Serial.begin(115200);
  Serial.println("Begin");
  // mpu6050.begin();
  // mpu6050.calcGyroOffsets(true);
  Dabble.begin("Lolin32Experiment");
  Serial.println("Started...");

  pinMode(pwmMotor1, OUTPUT);
  pinMode(dir1Motor1, OUTPUT);
  pinMode(dir2Motor1, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(pwmMotor1, pwmChannel);

  digitalWrite(dir1Motor1, HIGH);
  digitalWrite(dir2Motor1, LOW);
}

void loop() {
  // Serial.println("loop");
  // mpu6050.update();
  // accAngleX = mpu6050.getAccAngleX();
  // accAngleY = mpu6050.getAccAngleY();
  
  // float angleX = mpu6050.getAngleX();
  // float angleY = mpu6050.getAngleY();
  // float angleZ = mpu6050.getAngleZ();
  // long langle = angleX * 10l;
  // int motorSpeed = map(min(abs(langle), 900l), 0, 900, 0, 255);
  // if (angleX > 0) {
  //   digitalWrite(dir1Motor1, HIGH);
  //   digitalWrite(dir2Motor1, LOW);
  // } else {
  //   digitalWrite(dir1Motor1, LOW);
  //   digitalWrite(dir2Motor1, HIGH);
  // // }
  // ledcWrite(pwmChannel, motorSpeed);   
  if(millis() - timer > 250){
    // Serial.print("accAngleX/Y: "); Serial.print(accAngleX); Serial.print(" "); Serial.println(accAngleY);
    // Serial.print("gyroAngleX/Y:"); Serial.print(mpu6050.getGyroAngleX()); Serial.print(" "); Serial.println(mpu6050.getGyroAngleY());
    // Serial.print("angleX/Y"); Serial.print(mpu6050.getAngleX()); Serial.print(" "); Serial.println(mpu6050.getAngleY());
    // Serial.print("angleX: "); Serial.print(angleX); 
    // Serial.print(" angleY: "); Serial.print(angleY);
    // Serial.print(" angleZ: "); Serial.print(angleZ);
    // Serial.print(" speed "); Serial.println(motorSpeed);
    Serial.println();
    timer = millis();
  }
  
  // Dabble.processInput();
  // Controls.runMotor1(pwmMotor1,dir1Motor1,dir2Motor1);
  // while (Serial.available() != 0)
  // {
  //   Serialdata = String(Serialdata + char(Serial.read()));
  //   dataflag = 1;
  // }
  // if (dataflag == 1)
  // {
  //   Terminal.print(Serialdata);
  //   Serialdata = "";
  //   dataflag = 0;
  // }
  // if (Terminal.available() != 0)
  // {
  //   while (Terminal.available() != 0)
  //   {
  //     Serial.write(Terminal.read());
  //   }
  //   Serial.println();
  // }


  // if(millis() - timer > 250){
    // Terminal.print(mpu6050.getAngleX())
    
    // Serial.println("=======");
    // Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    // Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    // Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    // Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    // Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    // Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    // Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    // Terminal.print("accAngleX : ");Terminal.print(mpu6050.getAccAngleX());
    // Terminal.print("\taccAngleY : ");Terminal.println(mpu6050.getAccAngleY());
  
    // Terminal.print("gyroAngleX : ");Terminal.print(mpu6050.getGyroAngleX());
    // Terminal.print("\tgyroAngleY : ");Terminal.print(mpu6050.getGyroAngleY());
    // Terminal.print("\tgyroAngleZ : ");Terminal.println(mpu6050.getGyroAngleZ());
    
    // Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    // Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    // Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    // Serial.println("=====\n");
    // timer = millis();
    // 
  // }
}