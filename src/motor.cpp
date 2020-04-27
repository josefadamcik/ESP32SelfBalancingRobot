#include "motor.h"


void setupMPWM(int pinA1, int pinA2, int pinB1, int pinB2) {
  //documentation https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinA2);
  // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR_B1);
  // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, MOTOR_B2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, pinB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, pinB2);
  mcpwm_config_t pwm_config;
  pwm_config.frequency =5000;    //frequency,
  pwm_config.cmpr_a = 0;    		//duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    		//duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

}

void motorsForward(float speed) {
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void motorsReverse(float speed) {
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
 
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

void motorsGo(float speed) {
  if (speed < 0) {
    motorsReverse(speed * -1.0);
  } else {
    motorsForward(speed);
  }
}

void motorsStop() {
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
  // mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  // mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
}


