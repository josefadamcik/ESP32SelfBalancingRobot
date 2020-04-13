
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
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

String Serialdata;
bool dataflag = 0;
long timer = 0;
const uint8_t pwmMotor1 =  14;
const uint8_t dir1Motor1 = 12;
const uint8_t dir2Motor1 = 13;
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

void setup() {
  Wire.begin(21,22,400000); 
  Serial.begin(115200);
  Serial.println("Begin");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Dabble.begin("Lolin32Experiment");

  // pinMode(pwmMotor1, OUTPUT);
  // pinMode(dir1Motor1, OUTPUT);
  // pinMode(dir2Motor1, OUTPUT);
  // ledcSetup(pwmChannel, freq, resolution);
  // ledcAttachPin(pwmMotor1, pwmChannel);

  digitalWrite(dir1Motor1, HIGH);
  digitalWrite(dir2Motor1, LOW);
}

void loop() {
  // Serial.println("loop");
  // int motorSpeed = 255;
  // ledcWrite(pwmChannel, motorSpeed);   
  mpu6050.update();
  Dabble.processInput();
  Controls.runMotor1(pwmMotor1,dir1Motor1,dir2Motor1);
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


  if(millis() - timer > 500){
    
    Terminal.println("=======");
    // Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    // Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    // Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    // Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    // Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    // Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    // Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Terminal.print("accAngleX : ");Terminal.print(mpu6050.getAccAngleX());
    Terminal.print("\taccAngleY : ");Terminal.println(mpu6050.getAccAngleY());
  
    Terminal.print("gyroAngleX : ");Terminal.print(mpu6050.getGyroAngleX());
    Terminal.print("\tgyroAngleY : ");Terminal.print(mpu6050.getGyroAngleY());
    Terminal.print("\tgyroAngleZ : ");Terminal.println(mpu6050.getGyroAngleZ());
    
    Terminal.print("angleX : ");Terminal.print(mpu6050.getAngleX());
    Terminal.print("\tangleY : ");Terminal.print(mpu6050.getAngleY());
    Terminal.print("\tangleZ : ");Terminal.println(mpu6050.getAngleZ());
    Terminal.println("==================\n");
    timer = millis();
    
  }
}