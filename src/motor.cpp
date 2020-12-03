#include "motor.h"

#define WHEEL_RADIUS_MM 40.0
#define SPEED_MEASUREMENT_PERIOD_MS 10
#define PULSE_PER_REVOLUTION 451.74// 1:150 only one channel rising (3 * gearbox ratio)
#define USE_FAST_DECAY
#define PWM_FREQ 1000

const mcpwm_operator_t OPERATOR_FORWARD = MCPWM_OPR_A;
const mcpwm_operator_t OPERATOR_BACKWARD = MCPWM_OPR_B;

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

typedef struct {
    double rps; 
    double angularVelocity;  // rad / sec
    double velocity;         // mm / sec
} speed_info_t;

typedef volatile struct {
    gpio_num_t secondChannelPin;
    uint32_t direction;
    int64_t lastMeasurement;
    int64_t period;
} capture_speed_info_t;

static uint32_t lastSpeedMeasurementMs = 0;
static speed_info_t speedInfoA;
static speed_info_t speedInfoB;
static speed_info_t *speedInfo[2] = {&speedInfoA, &speedInfoB};
static volatile capture_speed_info_t DRAM_ATTR speedCaptureInfoA;
static volatile capture_speed_info_t DRAM_ATTR speedCaptureInfoB;


void computeSpeedInfoForChannel(capture_speed_info_t &captureSpeedInfo, speed_info_t &speedInfo, int64_t time) {
    int64_t period = captureSpeedInfo.period;
    if (time >= captureSpeedInfo.lastMeasurement + SPEED_MEASUREMENT_PERIOD_MS * 1000.0) {
        //if we got no new update for last 10ms say the speed is 0
        speedInfo.rps = 0;
        speedInfo.angularVelocity = 0;
        speedInfo.velocity = 0;
    } else {
        if (time > captureSpeedInfo.lastMeasurement + period) {
            //if the time since the last measurement is longer than the period, assume period is actually the difference
            period = time - captureSpeedInfo.lastMeasurement;
        }
        uint32_t timePerRotationWheel = (uint32_t)period * PULSE_PER_REVOLUTION; //us
        
        double sign = 1.0;
        if (captureSpeedInfo.direction == 1) {
            sign = -1.0;
        }
        double rpsWheel = sign * 1000000.0 / timePerRotationWheel;
        speedInfo.rps = rpsWheel;
        speedInfo.angularVelocity = M_TWOPI * rpsWheel;
        speedInfo.velocity = speedInfo.angularVelocity * WHEEL_RADIUS_MM;  // mm/s
    }
}

/** Don't run this too often. 10ms seems to be ok, less might increase error.*/
void computeSpeedInfo() {
    int64_t time = esp_timer_get_time();
    computeSpeedInfoForChannel(speedCaptureInfoA, speedInfoA, time);
    computeSpeedInfoForChannel(speedCaptureInfoB, speedInfoB, time);
}

double getAverageRps() { return (speedInfoA.rps + speedInfoB.rps) / 2;}

double getSpeedA() { return speedInfoA.velocity; }
double getSpeedB() { return speedInfoB.velocity; }


static void IRAM_ATTR speed_isr_handlerA() {
    int64_t time = esp_timer_get_time();
    uint32_t direction = gpio_get_level(speedCaptureInfoA.secondChannelPin);
    speedCaptureInfoA.period = time - speedCaptureInfoA.lastMeasurement;
    speedCaptureInfoA.lastMeasurement = time;
    speedCaptureInfoA.direction = direction;
}

static void IRAM_ATTR speed_isr_handlerB() {
    int64_t time = esp_timer_get_time();
    uint32_t direction = gpio_get_level(speedCaptureInfoB.secondChannelPin);
    speedCaptureInfoB.period = time - speedCaptureInfoB.lastMeasurement;
    speedCaptureInfoB.lastMeasurement = time;
    speedCaptureInfoB.direction = direction;
}

void setupEncoders(gpio_num_t sensA1, gpio_num_t sensA2, gpio_num_t sensB1, gpio_num_t sensB2) {
    pinMode(sensA1, INPUT_PULLUP);
    pinMode(sensA2, INPUT_PULLUP);
    pinMode(sensB1, INPUT_PULLUP);
    pinMode(sensB2, INPUT_PULLUP);
    speedCaptureInfoA.secondChannelPin = sensA2;
    attachInterrupt(digitalPinToInterrupt(sensA1), speed_isr_handlerA, RISING);
    speedCaptureInfoB.secondChannelPin = sensB2;
    attachInterrupt(digitalPinToInterrupt(sensB1), speed_isr_handlerB, RISING);
}

void setupMPWM(gpio_num_t pinA1, gpio_num_t pinA2, gpio_num_t pinB1, gpio_num_t pinB2) {
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