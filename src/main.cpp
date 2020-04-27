
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
#include "ArduinoOTA.h"
#include "keys.h"
#include "math.h"
#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_MODULE
#define INCLUDE_DABBLEINPUTS_MODULE
#include <DabbleESP32.h>
#include "motor.h"
#include "PID_v1.h"


MPU6050 mpu; 
#define MPU_INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

#define MOTOR_A1 32
#define MOTOR_A2 33
#define MOTOR_B1 26
#define MOTOR_B2 25
 

double targetAngle = 0;
double inputAngle;
double pidOutput;
double const initialPidKp=1, initialPidKi = 0, initialPikKd = 0; 
double pidKp=initialPidKp, pidKi=initialPidKi, pidKd=initialPikKd;

//Specify the links and initial tuning parameters
PID pid(&inputAngle, &pidOutput, &targetAngle, pidKp, pidKi, pidKd, DIRECT);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setupMPU6050() {  
  Serial.println(F("Initializing MPU6050"));
  mpu.initialize();
  pinMode(MPU_INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(142);
  mpu.setYGyroOffset(18);
  mpu.setZGyroOffset(5);
  mpu.setXAccelOffset(-3500);
  mpu.setYAccelOffset(-868);
  mpu.setZAccelOffset(1660);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    // Serial.println();
    // mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setupWifi() {
  Serial.println("Connecting to wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(MYSSID, MYPASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(2000);
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void setupOTA() {
  ArduinoOTA.setHostname("BalancingBotESP32");
  ArduinoOTA
    .onStart([]() {
      motorsStop();
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void waitForOTA() {
  Serial.println("Waiting for OTA for 10 sec");
  unsigned long start = millis();
  while(millis() - start <= 10000) {
      Serial.print(".");
      ArduinoOTA.handle();
      delay(500);
  }
}

void setupBluetooth() {
  Serial.println("Setub bluettooth");
  Dabble.begin("SelfBalancingBotisko");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  setupMPU6050();
  setupWifi();
  setupOTA();
  // waitForOTA();
  setupMPWM(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
  setupBluetooth();
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-60, 60);
  Serial.println("setup done");
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void processMPUData() {
  if (!dmpReady || !mpuInterrupt) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  static unsigned long lastOutput = 0;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // inputAngle = ypr[2];
    inputAngle = ypr[2] * 180 / M_PI;
    pid.Compute();
    // double speed = constrain(pidOutput, -100.0, 100.0);
    double speed = pidOutput;
    if (speed < 0) {
      speed = speed - 40;
    } else {
      speed = speed + 40;
    }
    motorsGo(speed);
    unsigned long now = millis();
    if (now - lastOutput > 250) {
      lastOutput = now;
      Serial.print("An: ");
      Serial.println(inputAngle);
      Serial.print("PID output: ");
      Serial.print(pidOutput);
      Serial.print(" speed ");
      Serial.println(speed);
    }
  }
}

static unsigned long loopSum = 0;
static unsigned int loopCount = 0;

void loop() {
  unsigned long loopStart = millis();
  static uint16_t lastPot1Value = 0;
  static uint16_t lastPot2Value = 0;
  uint8_t onOff = 0;
  ArduinoOTA.handle();

  Dabble.processInput();
  if (Dabble.isAppConnected()) {
    uint16_t pot1Value = Inputs.getPot1Value();
    uint16_t pot2Value = Inputs.getPot2Value();
    onOff = Inputs.getSlideSwitch1Value();
    if (pot1Value != lastPot1Value) {
      lastPot1Value = pot1Value;
      pidKp = pot1Value / 100.0;
      Serial.print("Changing KP: "); Serial.println(pidKp); 
      pid.SetTunings(pidKp, pidKi, pidKd);
    }
    if (pot2Value != lastPot2Value) {
      lastPot2Value = pot2Value;
      pidKi = pot2Value / 100.0;
      Serial.print("Changing KI: "); Serial.println(pidKi); 
      pid.SetTunings(pidKp, pidKi, pidKd);
    }
  }

  if (onOff > 0)  {
    processMPUData();
  } else {
    motorsStop();
  }

  loopSum += millis() - loopStart;
  loopCount++;
  if (loopSum >= 1000) {
    unsigned long average = loopSum / loopCount;
    Serial.print("Loops "); Serial.print(loopCount); Serial.print(" in "); Serial.println(loopSum);
    Serial.print("Average loop duration: "); 
    Serial.println(average);
    loopSum = 0;
    loopCount = 0;
  }
}

