#include "motor.h"

#define WHEEL_RADIUS_MM 40.0
#define SPEED_MEASUREMENT_PERIOD_MS 10
#define PULSE_PER_REVOLUTION 1807.0 //1:150 HP 6V motor
// #define PULSE_PER_REVOLUTION 2987.8 //1:150 HPCB 6B motor
#define USE_FAST_DECAY
#define PWM_FREQ 1000

const mcpwm_operator_t OPERATOR_FORWARD = MCPWM_OPR_A;
const mcpwm_operator_t OPERATOR_BACKWARD = MCPWM_OPR_B;

typedef struct {
    uint16_t pulseCount;
    double rps;
    double angularVelocity;  // rad / sec
    double velocity;         // mm / sec
} speed_info_t;

static uint32_t lastSpeedMeasurementMs = 0;
static speed_info_t speedInfoA;
static speed_info_t speedInfoB;
static speed_info_t *speedInfo[2] = {&speedInfoA, &speedInfoB};



uint32_t getPulseCount(pcnt_unit_t unit) {
    int16_t count = 0;
    pcnt_get_counter_value(unit, &count);
    return count;
}

void computeSpeedInfoForChannel(pcnt_unit_t unit, double timeCoef) {
    speedInfo[unit]->pulseCount = getPulseCount(unit);
    speedInfo[unit]->rps =
        speedInfo[unit]->pulseCount * timeCoef / PULSE_PER_REVOLUTION;
    speedInfo[unit]->angularVelocity = M_TWOPI * speedInfo[unit]->rps;
    speedInfo[unit]->velocity =
        speedInfo[unit]->angularVelocity * WHEEL_RADIUS_MM;  // mm/s
    // Serial.print("pulse "); Serial.print(unit); Serial.print(" ");
    // Serial.print(speedInfo[unit]->pulseCount); Serial.print(" rps ");
    // Serial.print(speedInfo[unit]->rps); Serial.print(" angular velocity ");
    // Serial.print(speedInfo[unit]->angularVelocity); 
    // Serial.print(" velocity "); Serial.print(speedInfo[unit]->velocity); Serial.println();
}

/** Don't run this too often. 10ms seems to be ok, less might increase error.*/
void computeSpeedInfo() {
    uint32_t now = millis();
    double timeCoef = 1000.0 / (now - lastSpeedMeasurementMs);  // per sec
    computeSpeedInfoForChannel(PCNT_UNIT_0, timeCoef);
    computeSpeedInfoForChannel(PCNT_UNIT_1, timeCoef);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    lastSpeedMeasurementMs = now;
}

double getAverageRps() { return (speedInfoA.rps + speedInfoB.rps) / 2;}

double getSpeedA() { return speedInfoA.velocity; }
double getSpeedB() { return speedInfoB.velocity; }

void setupPulseCounter(gpio_num_t pin, pcnt_unit_t unit,
                       pcnt_channel_t channel) {
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = -1,  // no controll pin
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP,  // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,  // Keep the primary counter mode if high
        // What to do on the positive / negative edge of pulse input?
        // increase on both
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_INC,
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = INT16_MAX,
        .counter_l_lim = 0,
        // unit/channel
        .unit = unit,
        .channel = channel,
    };

    pcnt_unit_config(&pcnt_config);
}
void setupPulseCounters(gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t sensB1,
                        gpio_num_t sensB2) {
    pinMode(sensA1, INPUT);
    pinMode(sensA2, INPUT);
    pinMode(sensB1, INPUT);
    pinMode(sensB2, INPUT);
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

void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t pinB1,
               gpio_num_t pinB2) {
    // documentation
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinA1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinA2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pinB1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, pinB2);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQ;  // frequency,
    pwm_config.cmpr_a = 0;            // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;            // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // PWM
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,
               &pwm_config);  // Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0,
               &pwm_config);  // Configure PWM1A & PWM1B with above settings
}

void motorDirectionStop(mcpwm_unit_t unit, mcpwm_operator_t mcpwmOperator) {
    mcpwm_set_duty(unit, MCPWM_TIMER_0, mcpwmOperator, 0.0);
    mcpwm_set_signal_low(unit, MCPWM_TIMER_0, mcpwmOperator);
}

void motorDirectionGo(mcpwm_unit_t unit, mcpwm_operator_t mcpwmOperator,
                      float speed) {
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

void motorsGo(float speedLeft, float speedRight) {
    motorGo(0, speedLeft);
    motorGo(1, speedRight);
}

void motorsStop() {
    motorDirectionStop(MCPWM_UNIT_0, OPERATOR_FORWARD);
    motorDirectionStop(MCPWM_UNIT_0, OPERATOR_BACKWARD);
    motorDirectionStop(MCPWM_UNIT_1, OPERATOR_FORWARD);
    motorDirectionStop(MCPWM_UNIT_1, OPERATOR_BACKWARD);
}

void motorPrintDebug() {
    Serial.println("motor debug info");
    Serial.print("U0/F: ");
    Serial.print(mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, OPERATOR_FORWARD));
    Serial.print(" U0/B: ");
    Serial.print(mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, OPERATOR_BACKWARD));
    Serial.print(" U1/F: ");
    Serial.print(mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, OPERATOR_FORWARD));
    Serial.print(" U1/B: ");
    Serial.print(mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, OPERATOR_BACKWARD));
    Serial.println();
}

void printSpeedInfoToSerial() {
    Serial.print("Measured speed A: ");
    Serial.print(speedInfoA.velocity);
    Serial.print(" mm/s; angular velocity: ");
    Serial.print(speedInfoA.angularVelocity);
    Serial.print(" rad/s; RPS: ");
    Serial.println(speedInfoA.rps);
    Serial.print("Measured speed B: ");
    Serial.print(speedInfoB.velocity);
    Serial.print(" mm/s; angular velocity: ");
    Serial.print(speedInfoB.angularVelocity);
    Serial.print(" rad/s; RPS: ");
    Serial.println(speedInfoB.rps);
    Serial.print(" Diff A-B: ");
    Serial.print(speedInfoA.velocity - speedInfoB.velocity);
    Serial.print(" angular velocity:");
    Serial.print(speedInfoA.angularVelocity - speedInfoB.angularVelocity);
    Serial.println();
}