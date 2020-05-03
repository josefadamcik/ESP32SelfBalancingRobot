#include "motor.h"


#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

volatile uint32_t counterA1 = 0;
volatile uint32_t counterA2 = 0;
volatile uint32_t counterB1 = 0;
volatile uint32_t counterB2 = 0;

uint32_t getRealTimeClockABPFreq() {
  return rtc_clk_apb_freq_get();
}

uint32_t getSpeedA() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  pcnt_counter_clear(PCNT_UNIT_0);
  return count;
}
uint32_t getSpeedB() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_1, &count);
  pcnt_counter_clear(PCNT_UNIT_1);
  return count;
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


void setupPulseCounter(gpio_num_t pin, pcnt_unit_t unit, pcnt_channel_t channel) {
  pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = -1, //no controll pin
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // What to do on the positive / negative edge of pulse input?
        //increase on both
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_INC,
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = 0,
        //unit/channel
        .unit = unit,
        .channel = channel,
    };

    pcnt_unit_config(&pcnt_config);
    
}
void setupPulseCounters(gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t sensB1, gpio_num_t sensB2) {
  setupPulseCounter(sensA1, PCNT_UNIT_0, PCNT_CHANNEL_0);
  setupPulseCounter(sensA2, PCNT_UNIT_0, PCNT_CHANNEL_1);
  setupPulseCounter(sensB1, PCNT_UNIT_1, PCNT_CHANNEL_0);
  setupPulseCounter(sensB2, PCNT_UNIT_1, PCNT_CHANNEL_1);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);
}


void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t pinB1, gpio_num_t pinB2, gpio_num_t sensB1, gpio_num_t sensB2) {
  //documentation https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinA2);
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, sensA1);
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, sensA2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, pinB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, pinB2);
  // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_0, sensB1);
  // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM_CAP_1, sensB2);
  // gpio_pulldown_en(sensA1);
  // gpio_pulldown_en(sensA2);
  // gpio_pulldown_en(sensB1);
  // gpio_pulldown_en(sensB2); 

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
  // mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  // mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  // mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  // mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
  
  //enable interrupt, so each this a rising edge occurs interrupt is triggered
  // MCPWM0.int_ena.val = CAP0_INT_EN | CAP1_INT_EN;
  // MCPWM1.int_ena.val = CAP0_INT_EN | CAP1_INT_EN;
  // mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
  // mcpwm_isr_register(MCPWM_UNIT_1, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

  setupPulseCounters(sensA1, sensA2, sensB1, sensB2);
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


