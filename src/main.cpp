#include "Arduino.h"
#include "Wire.h"
#include "ArduinoOTA.h"
#include "keys.h"
#include "math.h"
#include "motor.h"
#include "remote.h"
#include "imu.h"
#include "pid.h"

//IMU 
#define MPU_INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define PREFERENCES_NAMESPACE "app"
#define DEBUG_PID
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static volatile bool mpuInterrupt = false;
static void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

//MOTORS
#define MOTOR_A1 GPIO_NUM_32
#define MOTOR_A2 GPIO_NUM_33
#define MOTOR_B1 GPIO_NUM_26
#define MOTOR_B2 GPIO_NUM_25
#define MOTORA_S1 GPIO_NUM_5
#define MOTORA_S2 GPIO_NUM_17
#define MOTORB_S1 GPIO_NUM_4
#define MOTORB_S2 GPIO_NUM_16

double const initialPidKp=4, initialPidKi = 0, initialPikKd = 0; 
double const initialTargetAngle = 0;

float rPidKpEdit;  // 32767.. +32767 
float rPidKiEdit;  // 32767.. +32767 
float rPidKdEdit;  // 32767.. +32767 
int8_t rPidKp; // =0..100 slider position 
int8_t rPidKi; // =0..100 slider position 
int8_t rPidKd; // =0..100 slider position 
bool enginesOn = false;
bool pidOn = false;
int8_t speedLimit = 70;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

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
  RemoteXY_Init();
  RemoteXY.pidKp = 50;
  RemoteXY.pidKpEdit = initialPidKp;
  RemoteXY.pidKi = 50;
  RemoteXY.pidKiEdit = initialPidKi;
  RemoteXY.pidKd = 50;
  RemoteXY.pidKdEdit = initialPikKd;
  RemoteXY.motorLimit = speedLimit;
  RemoteXY.target = initialTargetAngle;
  sprintf(RemoteXY.motorLimitOut, "%d",speedLimit);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);

  setupMPU6050(MPU_INTERRUPT_PIN, PREFERENCES_NAMESPACE, dmpDataReady);
  // setupWifi();
  // setupOTA();
  // waitForOTA();
  setupMPWM(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
  setupPulseCounters(MOTORA_S1, MOTORA_S2, MOTORB_S1, MOTORB_S2);
  computeSpeedInfo();
  setupBluetooth();
  Serial.println("setup done");
}

//PID / SPEED
double targetAngle = initialTargetAngle;
double inputAngle;
double prevError = 0;
double speed;

static double pidKp=initialPidKp, pidKi=initialPidKi, pidKd=initialPikKd;

static double errorSum = 0;
static double pidOutput = 0;
static uint32_t lastPidSample = 0;
static double lastSampleTime;

void executePid() {
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
}

void printPidDebug() {
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
      // printSpeedInfoToSerial();
      motorPrintDebug();
      #endif

      sprintf(RemoteXY.txtCalibrate, "%f %f", inputAngle, pidOutput);
    }
}

void processMPUData() {
  if (!isMPUReady() || !mpuInterrupt) return;
  mpuInterrupt = false;

  if (getYPR(ypr)) { 
    inputAngle = ypr[2] * 180 / M_PI;
    executePid();
    double pidSpeed = constrain(pidOutput, -speedLimit, speedLimit);
    if (enginesOn && RemoteXY.pidOn) {
      if (abs(speed - pidSpeed) >= 0.1) {
        motorsGo(pidSpeed);
        speed = pidSpeed;
      }
    }
    printPidDebug();
  }
}

void loop() {
  static boolean calibrateOnNextLoop = false;
  ArduinoOTA.handle();
  computeSpeedInfo();
  processMPUData();

  // read controll data
  RemoteXY_Handler();
  enginesOn = RemoteXY.motorsOn == 1;
  bool newPidOn = enginesOn && RemoteXY.pidOn;
  if (!pidOn && newPidOn) {
    errorSum = 0;
    prevError = 0;
    pidOutput = 0;
    speed = 0;
  }
  pidOn = newPidOn;

  if (calibrateOnNextLoop) {
    calibrateOnNextLoop = false;
    calibrateMPU(PREFERENCES_NAMESPACE);
    // motorTest(speedLimit);
    // motorsTest(speedLimit);
  } else {
    if (RemoteXY.buttonCalibrate) {
      calibrateOnNextLoop = true;
    }
  }

  //check PID config
  if (RemoteXY.pidKdEdit != rPidKdEdit 
    || RemoteXY.pidKpEdit != rPidKpEdit
    || RemoteXY.pidKiEdit != rPidKiEdit) {
      rPidKpEdit = RemoteXY.pidKpEdit;
      rPidKiEdit = RemoteXY.pidKiEdit;
      rPidKdEdit = RemoteXY.pidKdEdit;
      pidKp = rPidKpEdit;
      pidKi = rPidKiEdit;
      pidKd = rPidKdEdit;
  }
  if (RemoteXY.target != targetAngle) {
    targetAngle = RemoteXY.target;
  }

  if (RemoteXY.motorLimit != speedLimit) {
    speedLimit = RemoteXY.motorLimit;
    sprintf(RemoteXY.motorLimitOut, "%d",speedLimit);
  }
  
  if (!pidOn) {
      speed = RemoteXY.joystickA_y;
      speed = map(speed, -100, 100, -speedLimit, speedLimit);
      if (enginesOn) {
        motorsGo(speed);
      }
  }
  if (!enginesOn) {
    motorsStop();
  }

  RemoteXY.ledState_g = enginesOn ? 255: 0;
  RemoteXY.ledState_r = !enginesOn ? 255: 0;
  RemoteXY.angle = map(inputAngle, -90, 90, 0, 100);
  RemoteXY.graph_var1 = prevError;
  RemoteXY.graph_var2 = speed;
  RemoteXY.graph_var3 = pidOutput;
  RemoteXY.speed = map(speed, -100, 100, 0, 100);
  if (calibrateOnNextLoop) {
    RemoteXY.ledBallance_r = 0;
    RemoteXY.ledBallance_g = 0;
    RemoteXY.ledBallance_b = 255;
  } else if (abs(inputAngle) < 2) {
    RemoteXY.ledBallance_r = 0;
    RemoteXY.ledBallance_g = 255;
    RemoteXY.ledBallance_b = 0;
  } else if (abs(inputAngle) < 8) {
    RemoteXY.ledBallance_r = 255;
    RemoteXY.ledBallance_g = 255;
    RemoteXY.ledBallance_b = 0;
  } else {
    RemoteXY.ledBallance_r = 255;
    RemoteXY.ledBallance_g = 0;
    RemoteXY.ledBallance_b = 0;
  }
}

