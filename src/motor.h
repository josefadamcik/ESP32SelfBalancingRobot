#ifndef MOTOR_H
#define MOTOR_H

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"



void setupMPWM(int pinA1, int pinA2, int pinB1, int pinB2);
void motorsGo(float speed);
void motorsStop();

#endif