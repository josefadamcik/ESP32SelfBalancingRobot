#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/rtc.h"
#include "driver/pcnt.h"

void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t pinB1, gpio_num_t pinB2);
void setupPulseCounters(gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t sensB1, gpio_num_t sensB2);
void motorsGo(float speedLeft, float speedRight);
void motorGo(uint8_t motor, float speed);
void motorsStop();
void computeSpeedInfo();
double getAverageRps();
double getSpeedA();
double getSpeedB();
void printSpeedInfoToSerial();
void motorPrintDebug();

#endif