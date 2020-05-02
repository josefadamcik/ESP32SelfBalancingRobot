#ifndef MOTOR_H
#define MOTOR_H

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"

void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t pinB1, gpio_num_t pinB2, gpio_num_t sensB1, gpio_num_t sensB2);
void motorsGo(float speed);
void motorsStop();
uint32_t getSpeedA1();
uint32_t getSpeedA2();
uint32_t getSpeedB1();
uint32_t getSpeedB2();

#endif