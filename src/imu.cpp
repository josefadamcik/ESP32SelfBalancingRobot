#include "imu.h"

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Preferences.h"

// MPU control/status vars
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
MPU6050 mpu;
bool dmpReady = false;   // set true if DMP init was successful
uint8_t fifoBuffer[64];  // FIFO storage buffer
Quaternion q;            // [w, x, y, z]         quaternion container
VectorFloat gravity;     // [x, y, z]            gravity vector

bool isMPUReady() { return dmpReady; }
bool getYPR(float *data) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(data, &q, &gravity);
        return true;
    }
    return false;
}

void setupMPU6050(const uint8_t interruptPin, const char *preferencesNamespace,
                  void (*dmpReadyIsr)(void)) {
    Serial.println(F("Initializing MPU6050"));
    mpu.initialize();
    pinMode(interruptPin, INPUT);
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // load offsets from preferences
    Preferences preferences;
    preferences.begin(preferencesNamespace, true);
    mpu.setXGyroOffset(preferences.getShort("gyro_x", 0));
    mpu.setYGyroOffset(preferences.getShort("gyro_y", 0));
    mpu.setZGyroOffset(preferences.getShort("gyro_z", 0));
    mpu.setXAccelOffset(preferences.getShort("accel_x", 0));
    mpu.setYAccelOffset(preferences.getShort("accel_y", 0));
    mpu.setZAccelOffset(preferences.getShort("accel_z", 0));
    preferences.end();
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(interruptPin), dmpReadyIsr,
                        RISING);
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

void calibrateMPU(const char *preferencesNamespace) {
    Serial.println("Calibrate MPU");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Preferences preferences;
    preferences.begin(preferencesNamespace, false);
    preferences.putShort("gyro_x", mpu.getXGyroOffset());
    preferences.putShort("gyro_y", mpu.getYGyroOffset());
    preferences.putShort("gyro_z", mpu.getZGyroOffset());
    preferences.putShort("accel_x", mpu.getXAccelOffset());
    preferences.putShort("accel_y", mpu.getYAccelOffset());
    preferences.putShort("accel_z", mpu.getZAccelOffset());
    preferences.end();
}
