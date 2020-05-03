#include "motor.h"


#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

#define WHEEL_RADIUS_MM 40.0
#define SPEED_MEASUREMENT_PERIOD_MS 50
#define PULSE_PER_REVOLUTION 1800.0
#define USE_FAST_DECAY

const mcpwm_operator_t OPERATOR_FORWARD = MCPWM_OPR_A;
const mcpwm_operator_t OPERATOR_BACKWARD = MCPWM_OPR_B;

typedef struct {
  uint16_t pulseCount;
  double rps;
  double angularVelocity; //rad / sec
  double velocity; // mm / sec
} speed_info_t;

static uint32_t lastSpeedMeasurementMs = 0;
static speed_info_t speedInfoA;
static speed_info_t speedInfoB;
static speed_info_t *speedInfo[2] = {&speedInfoA, &speedInfoB};



void printSpeedInfoToSerial() {
  Serial.print("Measured speed A: "); Serial.print(getSpeedA());
  Serial.print(" mm/s; angular velocity: ");
  Serial.print(speedInfoA.angularVelocity);
  Serial.print(" rad/s; RPS: ");
  Serial.println(speedInfoA.rps);
  Serial.print("Measured speed B: "); Serial.print(speedInfoB.velocity);
  Serial.print(" mm/s; angular velocity: ");
  Serial.print(speedInfoB.angularVelocity);
  Serial.print(" rad/s; RPS: ");
  Serial.println(speedInfoB.rps);
}

uint32_t getPulseCount(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  return count;
}

void computeSpeedInfoForChannel(pcnt_unit_t unit, double timeCoef) {

  speedInfo[unit]->pulseCount = getPulseCount(unit);
  speedInfo[unit]->rps = speedInfo[unit]->pulseCount * timeCoef / PULSE_PER_REVOLUTION;
  speedInfo[unit]->angularVelocity = M_TWOPI * speedInfo[unit]->rps;
  speedInfo[unit]->velocity = speedInfo[unit]->angularVelocity * WHEEL_RADIUS_MM;
  // Serial.println("pulse "); Serial.print(unit); Serial.print(" "); Serial.print(speedInfo[unit]->pulseCount);
  // Serial.print("rps "); Serial.print(speedInfo[unit]->pulseCount);
}

void computeSpeedInfo() {
  uint32_t now = millis();
  if (now - lastSpeedMeasurementMs > SPEED_MEASUREMENT_PERIOD_MS) {
    double timeCoef = 1000.0 / (now - lastSpeedMeasurementMs); //per sec
    computeSpeedInfoForChannel(PCNT_UNIT_0, timeCoef);
    computeSpeedInfoForChannel(PCNT_UNIT_1, timeCoef);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    lastSpeedMeasurementMs = now;
  }
}

double getSpeedA() {
  return speedInfoA.velocity;
}
double getSpeedB() {
  return speedInfoB.velocity;
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


void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t pinB1, gpio_num_t pinB2) {
  //documentation https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-rerence/peripherals/mcpwm.html
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinA1);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinA2);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pinB1);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, pinB2);

  mcpwm_config_t pwm_config;
  pwm_config.frequency =10000;    //frequency,
  pwm_config.cmpr_a = 0;    		//duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;    		//duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  //PWM
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);    //Configure PWM1A & PWM1B with above settings
}

void motorDirectionStop(mcpwm_unit_t unit, mcpwm_operator_t mcpwmOperator) {
  #if defined(USE_FAST_DECAY)
    mcpwm_set_signal_low(unit, MCPWM_TIMER_0, mcpwmOperator);
  #else 
    mcpwm_set_signal_hiph(unit, MCPWM_TIMER_0, mcpwmOperator);
  #endif
}

void motorDirectionGo(mcpwm_unit_t unit, mcpwm_operator_t mcpwmOperator, float speed) {
  mcpwm_set_duty(unit, MCPWM_TIMER_0, mcpwmOperator, speed);
  mcpwm_set_duty_type(unit, MCPWM_TIMER_0, mcpwmOperator, MCPWM_DUTY_MODE_0); 
}

void motorsForward(float speed) {
  motorDirectionStop(MCPWM_UNIT_0, OPERATOR_BACKWARD);
  motorDirectionStop(MCPWM_UNIT_1, OPERATOR_BACKWARD);
  motorDirectionGo(MCPWM_UNIT_0, OPERATOR_FORWARD, speed);
  motorDirectionGo(MCPWM_UNIT_1, OPERATOR_FORWARD, speed);
}

void motorsReverse(float speed) {
  motorDirectionStop(MCPWM_UNIT_0, OPERATOR_FORWARD);
  motorDirectionStop(MCPWM_UNIT_1, OPERATOR_FORWARD);
  motorDirectionGo(MCPWM_UNIT_0, OPERATOR_BACKWARD, speed);
  motorDirectionGo(MCPWM_UNIT_1, OPERATOR_BACKWARD, speed);
}

void motorGo(uint8_t motor, float speed) {
  mcpwm_unit_t selectedUnit = MCPWM_UNIT_0;
  if (motor == 1) {
    selectedUnit = MCPWM_UNIT_1;
  }
  if (speed < 0) {
    speed = speed * -1.0;
    motorDirectionStop(selectedUnit, OPERATOR_FORWARD);
    motorDirectionGo(selectedUnit, OPERATOR_BACKWARD, speed);
  } else {
    motorDirectionStop(selectedUnit, OPERATOR_BACKWARD);
    motorDirectionGo(selectedUnit, OPERATOR_FORWARD, speed);
  }
}

void motorsGo(float speed) {
  if (speed < 0) {
    motorsReverse(speed * -1.0);
  } else {
    motorsForward(speed);
  }
}

void motorsStop() {
  motorDirectionStop(MCPWM_UNIT_0, OPERATOR_FORWARD);
  motorDirectionStop(MCPWM_UNIT_0, OPERATOR_BACKWARD);
  motorDirectionStop(MCPWM_UNIT_1, OPERATOR_FORWARD);
  motorDirectionStop(MCPWM_UNIT_1, OPERATOR_BACKWARD);
}


