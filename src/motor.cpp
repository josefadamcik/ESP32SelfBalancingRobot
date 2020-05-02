#include "motor.h"


#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

volatile uint32_t counterA1 = 0;
volatile uint32_t counterA2 = 0;
volatile uint32_t counterB1 = 0;
volatile uint32_t counterB2 = 0;

uint32_t getSpeedA1() {
  return mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
  // return counterA1;
}
uint32_t getSpeedA2() {
  return mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);
  // return counterA2;
}
uint32_t getSpeedB1() {
  return mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
  // return counterB1;
}
uint32_t getSpeedB2() {
  return mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP1);
  // return counterB2;
}


typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

 
void IRAM_ATTR isr_handler(void*)
{
    // uint32_t mcpwm_intr_status;
    // mcpwm_intr_status = MCPWM0.int_st.val; //Read interrupt status
    // if (mcpwm_intr_status & CAP0_INT_EN) { 
    //     counterA1 = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
    // }
    // if (mcpwm_intr_status & CAP1_INT_EN) { 
    //     counterA2 = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
    // }

    // mcpwm_intr_status = MCPWM1.int_st.val; //Read interrupt status
    // if (mcpwm_intr_status & CAP0_INT_EN) { 
    //     counterB1 = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0); //get capture signal counter value
    // }
    // if (mcpwm_intr_status & CAP1_INT_EN) { 
    //     counterB2 = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP1); //get capture signal counter value
    // }
}

void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t pinB1, gpio_num_t pinB2, gpio_num_t sensB1, gpio_num_t sensB2) {
  //documentation https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinA2);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, sensA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, sensA2);
  gpio_pulldown_en(sensA1);
  gpio_pulldown_en(sensA2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, pinB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, pinB2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, sensB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, sensB2);
  gpio_pulldown_en(sensB1);
  gpio_pulldown_en(sensB2); 

  mcpwm_config_t pwm_config;
  pwm_config.frequency =10000;    //frequency,
  pwm_config.cmpr_a = 0;    		//duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    		//duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  //PWM
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

  //Capture
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  
  //enable interrupt, so each this a rising edge occurs interrupt is triggered
  // MCPWM0.int_ena.val = CAP0_INT_EN | CAP1_INT_EN;
  // MCPWM1.int_ena.val = CAP0_INT_EN | CAP1_INT_EN;
  // mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
  // mcpwm_isr_register(MCPWM_UNIT_1, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

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


