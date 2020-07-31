#include "Arduino.h"
#include "ArduinoOTA.h"
#include "Wire.h"
#include "imu.h"
#include "keys.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote.h"

// IMU
#define MPU_INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define PREFERENCES_NAMESPACE "app"

static volatile bool DRAM_ATTR mpuInterrupt = false;

static void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

#define DEBUG_PRINT_PID
#define DEBUG_PRINT_SPEED
#define DEBUG_PRINT_STATE
// #define DEBUG_PRINT_LOOP_STAT

// MOTORS
#define MOTOR_A1 GPIO_NUM_33
#define MOTOR_A2 GPIO_NUM_32
#define MOTOR_B1 GPIO_NUM_25
#define MOTOR_B2 GPIO_NUM_26
#define MOTORA_S1 GPIO_NUM_4
#define MOTORA_S2 GPIO_NUM_16
#define MOTORB_S1 GPIO_NUM_5
#define MOTORB_S2 GPIO_NUM_17

struct {
    float imuYawPitchRoll[3];  // [yaw, pitch, roll]   yaw/pitch/roll container
                               // and gravity vector
    float rPidKpEdit;          // 32767.. +32767
    float rPidKiEdit;          // 32767.. +32767
    float rPidKdEdit;          // 32767.. +32767
    float rTargetEdit;

    int8_t rPidKp;  // =0..100 slider position
    int8_t rPidKi;  // =0..100 slider position
    int8_t rPidKd;  // =0..100 slider position
    int8_t rTargetAdjust;
    bool motorsEnabled = false;
    bool pidEnabled = false;
    int8_t dutyCycleLimit = 100;
    double dutyCycle;
    boolean calibrateOnNextLoop = false;
    int8_t throttle = 0;
    int8_t steering = 0;
    double inputAngle = 0;
    double targetAngle = 0;
    double targetAdjustFromThrottle = 0;
    double leftSpeedAdjustFromSteering = 0;
    double rightSpeedAdjustFromSteering = 0;
    double currentSpeed = 0;
} State;

// PID
const double targetAngleLimit = 5;
const double balancingAngleLimit = 10;

double const initialPidKp = 5, initialPidKi = 20, initialPikKd = 0.2;
double const initialTargetAngle = 0;
Pid pidAngle(initialPidKp, initialPidKi, initialPikKd, initialTargetAngle);

Pid pidSpeed(0.1, 0.1, 0, 0);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void printDebug() {
    static unsigned long lastOutput = 0;
    unsigned long now = millis();
    if (now - lastOutput > 250) {
        lastOutput = now;
#if defined(DEBUG_PRINT_PID)
        pidAngle.printDebug();
#endif
#if defined(DEBUG_PRINT_STATE)
        Serial.print("duty ");
        Serial.print(State.dutyCycle);
        Serial.print(" target angle ");
        Serial.print(State.targetAngle);
        Serial.print(" speed ");
        Serial.print(State.currentSpeed);
        // Serial.print(" throttle: ");
        // Serial.print(State.throttle);
        // Serial.print(" thrt adj: ");
        // Serial.print(State.targetAdjustFromThrottle);
        // Serial.print(" steering: ");
        // Serial.print(State.steering);
        // Serial.print(" steer left adj: ");
        // Serial.print(State.leftSpeedAdjustFromSteering);
        // Serial.print(" steer rigth adj: ");
        // Serial.print(State.rightSpeedAdjustFromSteering);
        Serial.println();
#endif
#if defined(DEBUG_PRINT_SPEED)
        printSpeedInfoToSerial();
        motorPrintDebug();
#endif
    }
}

void resetRometePidKSliders() {
    RemoteXY.pidKp = 50;
    RemoteXY.pidKi = 50;
    RemoteXY.pidKd = 50;
    State.rPidKp = RemoteXY.pidKp;
    State.rPidKi = RemoteXY.pidKi;
    State.rPidKd = RemoteXY.pidKd;
}

void setupBluetooth() {
    Serial.println("Setub bluettooth");
    RemoteXY_Init();
    RemoteXY.pidKpEdit = initialPidKp;
    RemoteXY.pidKiEdit = initialPidKi;
    RemoteXY.pidKdEdit = initialPikKd;
    resetRometePidKSliders();
    RemoteXY.motorLimit = State.dutyCycleLimit;
    sprintf(RemoteXY.motorLimitOut, "%d", State.dutyCycleLimit);
}

void setup() {
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock
    Serial.begin(115200);

    setupMPU6050(MPU_INTERRUPT_PIN, PREFERENCES_NAMESPACE, dmpDataReady);
    // setupWifi();
    // setupOTA();
    // waitForOTA();
    setupMPWM(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
    setupPulseCounters(MOTORA_S1, MOTORA_S2, MOTORB_S1, MOTORB_S2);
    setupBluetooth();
    Serial.println("setup done");
}

boolean processMPUData() {
    if (!isMPUReady() || !mpuInterrupt) return false;
    if (getYPR(State.imuYawPitchRoll)) {
        mpuInterrupt = false;
        State.inputAngle = State.imuYawPitchRoll[2] * 180 / M_PI;
        return true;
    }
    return false;
}

void ballance() {
    double targetSpeed = 0; //(rps)
    double currentSpeed = getAverageRps();
    if (State.dutyCycle < 0) {//fix the fact we always measure positive speed
        currentSpeed = -currentSpeed;
    }
    State.currentSpeed = currentSpeed;
    pidSpeed.setTarget(targetSpeed);
    State.targetAngle = pidSpeed.execute(currentSpeed);

    pidAngle.setTarget(State.targetAngle);
    double pidOutput = pidAngle.execute(State.inputAngle);
    //from here the speed is more like duty cycle for motor.
    double pidSpeed = constrain(pidOutput, -State.dutyCycleLimit, State.dutyCycleLimit);
    if (State.pidEnabled && State.motorsEnabled) {
        if (abs(State.dutyCycle - pidSpeed) >= 0.1) {
            motorsGo(State.dutyCycle + State.leftSpeedAdjustFromSteering,
                        State.dutyCycle + State.rightSpeedAdjustFromSteering);
            State.dutyCycle = pidSpeed;
        }
    }
}

void turnOnBallancing() {
    pidAngle.reset();
    State.dutyCycle = 0;
    State.throttle = 0;
    State.leftSpeedAdjustFromSteering = 0;
    State.rightSpeedAdjustFromSteering = 0;
    RemoteXY.joystickA_x = 0;
    RemoteXY.joystickA_y = 0;
}

void turnOffBalancing() {
    State.dutyCycle = 0;
    State.throttle = 0;
    State.leftSpeedAdjustFromSteering = 0;
    State.rightSpeedAdjustFromSteering = 0;
    RemoteXY.joystickA_x = 0;
    RemoteXY.joystickA_y = 0;
}

void updatePidEnabledFromRemote() {
    bool validAngle = abs(State.inputAngle) < balancingAngleLimit;
    bool shouldBeOn = State.motorsEnabled && RemoteXY.pidOn && validAngle;
    if (!State.pidEnabled && shouldBeOn) {
        turnOnBallancing();
    } else if (State.pidEnabled && !shouldBeOn) {
        turnOffBalancing();
    }
    State.pidEnabled = shouldBeOn;
}

void updateMotorsEnabledFromRemote() {
    State.motorsEnabled = RemoteXY.motorsOn == 1;
}

float adjustPidKoefProportionaly(float base, int8_t adjustment) {
    float coef = (adjustment - 50.0) / 50.0;  //-1..1
    // coef = coef * 0.1;                        // allow for -10 ... 10 %;
    //allo -+ 100%
    return base + (base * coef);
}

float adjustTargetProportinaly(float base, int8_t adjustment) {
    float coef = (adjustment - 50.0) / 50.0;  //-1..1
    coef = coef * 0.2;                        // allow for -20 ... 20 %;
    return base + (base * coef);
}

void updatePidTarget() {
    double newTarget =
        adjustTargetProportinaly(State.rTargetEdit, State.rTargetAdjust);
    newTarget += State.targetAdjustFromThrottle;
    pidAngle.setTarget(newTarget);
}

void updateConfigFromRemote() {
    boolean keofChanged = false;
    if (RemoteXY.pidKdEdit != State.rPidKdEdit ||
        RemoteXY.pidKpEdit != State.rPidKpEdit ||
        RemoteXY.pidKiEdit != State.rPidKiEdit) {
        State.rPidKpEdit = RemoteXY.pidKpEdit;
        State.rPidKiEdit = RemoteXY.pidKiEdit;
        State.rPidKdEdit = RemoteXY.pidKdEdit;
        resetRometePidKSliders();
        pidAngle.setKeoficients(State.rPidKpEdit, State.rPidKiEdit, State.rPidKdEdit);
        keofChanged = true;
    }

    if (RemoteXY.pidKp != State.rPidKp || RemoteXY.pidKi != State.rPidKi ||
        RemoteXY.pidKd != State.rPidKd) {
        State.rPidKp = RemoteXY.pidKp;
        State.rPidKi = RemoteXY.pidKi;
        State.rPidKd = RemoteXY.pidKd;
        pidAngle.setKeoficients(
            adjustPidKoefProportionaly(State.rPidKpEdit, State.rPidKp),
            adjustPidKoefProportionaly(State.rPidKiEdit, State.rPidKi),
            adjustPidKoefProportionaly(State.rPidKdEdit, State.rPidKd));
        keofChanged = true;
    }

    if (keofChanged) {
        sprintf(RemoteXY.pidKpVal, "%f", pidAngle.kp);
        sprintf(RemoteXY.pidKiVal, "%f", pidAngle.ki);
        sprintf(RemoteXY.pidKdVal, "%f", pidAngle.kd);
    }

    if (RemoteXY.motorLimit != State.dutyCycleLimit) {
        State.dutyCycleLimit = RemoteXY.motorLimit;
        sprintf(RemoteXY.motorLimitOut, "%d", State.dutyCycleLimit);
    }
}

void updateDataForRemote() {
    RemoteXY.ledState_g = State.motorsEnabled ? 255 : 0;
    RemoteXY.ledState_r = !State.motorsEnabled ? 255 : 0;
    RemoteXY.graph_var1 = pidAngle.prevError;
    RemoteXY.graph_var2 = pidAngle.output;
    // RemoteXY.graph_var3 = pidOutput;
    RemoteXY.speedGraph_var1 = State.dutyCycle;
    RemoteXY.speedGraph_var2 = getSpeedA();
    RemoteXY.speedGraph_var3 = getSpeedB();
    RemoteXY.speedGraph_var4 = getSpeedA() - getSpeedB();
    RemoteXY.speed = map(State.dutyCycle, -100, 100, 0, 100);
    if (State.calibrateOnNextLoop) {
        RemoteXY.ledBallance_r = 0;
        RemoteXY.ledBallance_g = 0;
        RemoteXY.ledBallance_b = 255;
    } else if (abs(pidAngle.input - State.targetAngle) < 1) {
        RemoteXY.ledBallance_r = 0;
        RemoteXY.ledBallance_g = 255;
        RemoteXY.ledBallance_b = 0;
    } else if (abs(pidAngle.input - State.targetAngle) < 5) {
        RemoteXY.ledBallance_r = 255;
        RemoteXY.ledBallance_g = 255;
        RemoteXY.ledBallance_b = 0;
    } else if (abs(pidAngle.input - State.targetAngle) < 10) {
        RemoteXY.ledBallance_r = 255;
        RemoteXY.ledBallance_g = 128;
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

void handleSteering() {
    if (State.steering != RemoteXY.joystickA_x) {
        State.steering = RemoteXY.joystickA_x;
        State.leftSpeedAdjustFromSteering = -1 * State.steering * 0.05;
        State.rightSpeedAdjustFromSteering = State.steering * 0.05;
    }
}

void handleThrottle() {
    // if (State.pidEnabled) {
    //     if (State.throttle != RemoteXY.joystickA_y) {
    //         State.throttle = RemoteXY.joystickA_y;
    //         State.targetAdjustFromThrottle = State.throttle * 0.04;
    //         updatePidTarget();
    //     }
    // } else {
    //     State.targetAdjustFromThrottle = 0;
    //     State.dutyCycle = RemoteXY.joystickA_y;
    //     State.dutyCycle =
    //         map(State.dutyCycle, -100, 100, -State.dutyCycleLimit, State.dutyCycleLimit);
    //     if (State.motorsEnabled) {
    //         motorsGo(State.dutyCycle + State.leftSpeedAdjustFromSteering,
    //                  State.dutyCycle + State.rightSpeedAdjustFromSteering);
    //     }
    // }
}

void computeLoopTime() {
    static long lastLoopTime = 0;
    static long lastLoopUsedTime = 0;
    static unsigned long loopStartTime = 0;
    static float deltaTime = 0;  // unit: seconds
    lastLoopUsedTime = micros() - loopStartTime;
    lastLoopTime = micros() - loopStartTime;
    deltaTime = (float)lastLoopTime / (float)1000000;
    loopStartTime = micros();
#ifdef DEBUG_PRINT_LOOP_STAT
    Serial.println(lastLoopUsedTime);
#endif
}

void loop() {
    // ArduinoOTA.handle();
    RemoteXY_Handler();
    boolean processingTriggered = processMPUData();
    if (processingTriggered) {
        updateMotorsEnabledFromRemote();
        updatePidEnabledFromRemote();

        computeSpeedInfo();
        ballance();

        handleCalibration();
        updateConfigFromRemote();

        // handleSteering();
        handleThrottle();

        if (!State.pidEnabled || !State.motorsEnabled) {
            motorsStop();
        }
        // if (!State.motorsEnabled) {
            // motorsStop();
        // }

        updateDataForRemote();
        printDebug();
        computeLoopTime();
    }
}