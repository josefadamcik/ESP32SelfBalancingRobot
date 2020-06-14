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

static volatile bool DRAM_ATTR mpuInterrupt = false;

static void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

#define DEBUG_PID

//MOTORS
#define MOTOR_A1 GPIO_NUM_33
#define MOTOR_A2 GPIO_NUM_32
#define MOTOR_B1 GPIO_NUM_25
#define MOTOR_B2 GPIO_NUM_26
#define MOTORA_S1 GPIO_NUM_4
#define MOTORA_S2 GPIO_NUM_16
#define MOTORB_S1 GPIO_NUM_5
#define MOTORB_S2 GPIO_NUM_17


struct {
  float imuYawPitchRoll[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  float rPidKpEdit;  // 32767.. +32767 
  float rPidKiEdit;  // 32767.. +32767 
  float rPidKdEdit;  // 32767.. +32767 
  int8_t rPidKp; // =0..100 slider position 
  int8_t rPidKi; // =0..100 slider position 
  int8_t rPidKd; // =0..100 slider position 
  bool motorsEnabled = false;
  bool pidEnabled = false;
  int8_t speedLimit = 70;
  double speed;
  boolean calibrateOnNextLoop = false;
} State;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void printDebug() {
    static unsigned long lastOutput = 0;
    unsigned long now = millis();
    if (now - lastOutput > 250) {
      lastOutput = now;
      #if defined(DEBUG_PID)
      pidPrintDebug();
      Serial.print("speed "); Serial.println(State.speed);
      printSpeedInfoToSerial();
      motorPrintDebug();
      #endif
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
  RemoteXY.motorLimit = State.speedLimit;
  RemoteXY.target = initialTargetAngle;
  sprintf(RemoteXY.motorLimitOut, "%d",State.speedLimit);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
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

void processMPUData() {
  if (!isMPUReady() || !mpuInterrupt) return;
  mpuInterrupt = false;

  if (getYPR(State.imuYawPitchRoll)) { 
    double inputAngle = State.imuYawPitchRoll[2] * 180 / M_PI;
    double pidOutput = pidExecute(inputAngle);
    double pidSpeed = constrain(pidOutput, -State.speedLimit, State.speedLimit);
    if (State.motorsEnabled && RemoteXY.pidOn) {
      if (abs(State.speed - pidSpeed) >= 0.1) {
        motorsGo(pidSpeed);
        State.speed = pidSpeed;
      }
    }
    printDebug();
  }
}

void updateConfigFromRemote() {
  if (RemoteXY.pidKdEdit != State.rPidKdEdit 
    || RemoteXY.pidKpEdit != State.rPidKpEdit
    || RemoteXY.pidKiEdit != State.rPidKiEdit) {
      State.rPidKpEdit = RemoteXY.pidKpEdit;
      State.rPidKiEdit = RemoteXY.pidKiEdit;
      State.rPidKdEdit = RemoteXY.pidKdEdit;
      pidSetKeoficients(State.rPidKpEdit, State.rPidKiEdit, State.rPidKdEdit);
  }
  if (RemoteXY.target != Pid.target) {
    pidSetTarget(RemoteXY.target);
  }

  if (RemoteXY.motorLimit != State.speedLimit) {
    State.speedLimit = RemoteXY.motorLimit;
    sprintf(RemoteXY.motorLimitOut, "%d",State.speedLimit);
  }
}

void updateDataForRemote() {
  RemoteXY.ledState_g = State.motorsEnabled ? 255: 0;
  RemoteXY.ledState_r = !State.motorsEnabled ? 255: 0;
  RemoteXY.angle = map(Pid.input, -90, 90, 0, 100);
  RemoteXY.graph_var1 = Pid.prevError;
  RemoteXY.graph_var2 = Pid.output;
  // RemoteXY.graph_var3 = pidOutput;
  RemoteXY.speedGraph_var1 = State.speed;
  RemoteXY.speedGraph_var2 = getSpeedA();
  RemoteXY.speedGraph_var3 = getSpeedB();
  RemoteXY.speed = map(State.speed, -100, 100, 0, 100);
  if (State.calibrateOnNextLoop) {
    RemoteXY.ledBallance_r = 0;
    RemoteXY.ledBallance_g = 0;
    RemoteXY.ledBallance_b = 255;
  } else if (abs(Pid.input) < 2) {
    RemoteXY.ledBallance_r = 0;
    RemoteXY.ledBallance_g = 255;
    RemoteXY.ledBallance_b = 0;
  } else if (abs(Pid.input) < 8) {
    RemoteXY.ledBallance_r = 255;
    RemoteXY.ledBallance_g = 255;
    RemoteXY.ledBallance_b = 0;
  } else {
    RemoteXY.ledBallance_r = 255;
    RemoteXY.ledBallance_g = 0;
    RemoteXY.ledBallance_b = 0;
  }
}

void handleCalibration() {
  if (State.calibrateOnNextLoop) {
    State.calibrateOnNextLoop = false;
    calibrateMPU(PREFERENCES_NAMESPACE);
  } else {
    if (RemoteXY.buttonCalibrate) {
      State.calibrateOnNextLoop = true;
    }
  }
}

void loop() {

  // ArduinoOTA.handle();
  computeSpeedInfo();
  processMPUData();
  RemoteXY_Handler();

  State.motorsEnabled = RemoteXY.motorsOn == 1;
  bool newPidOn = State.motorsEnabled && RemoteXY.pidOn;
  if (!State.pidEnabled && newPidOn) {
    pidReset();
    State.speed = 0;
  }
  State.pidEnabled = newPidOn;

  handleCalibration();
  updateConfigFromRemote();
  
  if (!State.pidEnabled) {
      State.speed = RemoteXY.joystickA_y;
      State.speed = map(State.speed, -100, 100, -State.speedLimit, State.speedLimit);
      if (State.motorsEnabled) {
        // motorGo(0, speed);
        motorsGo(State.speed);
      }
  }
  if (!State.motorsEnabled) {
    motorsStop();
  }

  updateDataForRemote();
}

