// Copyright 2017 Jordan Ford

#include "Arduino.h"
#include "vehicle.h"
#include "lookup.h"

/********** Global Variables **********/
volatile long long odom = 0;
volatile double target_speed_tps_ = 0;
volatile double actual_speed_tps_ = 0;
volatile double sent_speed_tps_ = 0;
volatile uint16_t throttle_val_ = THROTTLE_STOP;
IntervalTimer odom_timer;

/********** Constructor **********/
Vehicle::Vehicle(void) : steering_val_(STEERING_CENTER)
{
    // Set up PWM
    analogWriteResolution(PWM_RESOLUTION_BITS);
    analogWriteFrequency(THROTTLE_PIN, THROTTLE_PWM_FREQUENCY_HZ);
    analogWriteFrequency(STEERING_PIN, STEERING_PWM_FREQUENCY_HZ);
    
    // Set steering to the center and throttle off
    analogWrite(STEERING_PIN, STEERING_CENTER);
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Attach ESTOP
    pinMode(ESTOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
    
    // Attach motor hall effect sensor interrupts
    pinMode(MOTOR_PHASE0_PIN, INPUT);
    pinMode(MOTOR_PHASE1_PIN, INPUT);
    pinMode(MOTOR_PHASE2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE0_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE1_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE2_PIN), odom_isr, CHANGE);
    odom_timer.begin(odom_isr, 20000);
}

/********** Destructor **********/
Vehicle::~Vehicle(void) {}

/********** E-Stop Interrupt Routine **********/
void Vehicle::estop_isr(void) {
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    analogWrite(STEERING_PIN, STEERING_CENTER);
    Serial.write('e');
    Serial.write('\n');
    error_blink(ESTOP_ERR_PERIOD);
}

void Vehicle::error_blink(int period_ms) {
    while(1) {
        digitalWriteFast(LED_PIN, LOW);
        delay(period_ms>>1);
        digitalWriteFast(LED_PIN, HIGH);
        delay(period_ms>>1);
    }
}

/********** Hall Effect Sensors Interrupt Routine **********/
void Vehicle::odom_isr() {
    static int prev_state = -1;
    uint32_t curr_time = micros();
    static uint32_t last_time = curr_time;

    int motor_seq_num = (digitalRead(MOTOR_PHASE0_PIN)<<2 | 
                         digitalRead(MOTOR_PHASE1_PIN)<<1 | 
                         digitalRead(MOTOR_PHASE2_PIN)) - 1;

    int state = hall_effect_sequence[motor_seq_num];
    if (prev_state < 0) {
        prev_state = state;
        return;
    }
  
    if (prev_state == state) {
        if (curr_time - last_time > 19000) { actual_speed_tps_ = 0.0; }
        else { return; }
    }
    else if (state-1 == prev_state || (state == 0 && prev_state == 5)) { 
        odom++;
        actual_speed_tps_ = (float)1e6/(curr_time - last_time);
    }
    else if (state+1 == prev_state || (state == 5 && prev_state == 0)) { 
        odom--;
        actual_speed_tps_ = (float)-1e6/(curr_time - last_time);
    }
    else { estop_isr(); }

    // PID on throttle value
    float err = target_speed_tps_ - actual_speed_tps_;
    static float err_prev = err;
    static float err_int = 0;

    sent_speed_tps_ -= KP*err + KI*err_int + KD*(err-err_prev);
    send_speed_(sent_speed_tps_);

    // Update
    err_int = 0.9*err_int + 0.1*err;
    err_prev = err;
    last_time = curr_time;
    prev_state = state;
}

/********** Public Functions : Setters **********/
void Vehicle::set_steering_angle(float angle_rad) {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;
    steering_val_ = map( angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD, STEERING_RIGHT_MAX, STEERING_LEFT_MAX );
    analogWrite(STEERING_PIN, steering_val_);
}

void Vehicle::set_throttle_speed(float speed_mps) {
    target_speed_tps_ = speed_mps * 1000.0 / (2*M_PI*VEHICLE_WHEEL_RAD_MM) * VEHICLE_GEAR_RATIO * 6;
    sent_speed_tps_ = target_speed_tps_;
    send_speed_(target_speed_tps_);
}

void Vehicle::run_profile(int profile) {
}

/********** Public Functions : Getters **********/
double Vehicle::get_odom_m(void) {
    return ( odom / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ; 
}

double Vehicle::get_speed_tps(void) {
    return actual_speed_tps_;
}

double Vehicle::get_speed_mps(void) {
    return ( actual_speed_tps_ / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ; 
}

/********** Private Functions **********/
void Vehicle::send_speed_(float speed_tps) {
    // Cap speed at the vehicle limits.
    speed_tps = (speed_tps > MAX_THROTTLE_SPEED_TPS) ? MAX_THROTTLE_SPEED_TPS : speed_tps;
    speed_tps = (speed_tps < MIN_THROTTLE_SPEED_TPS) ? MIN_THROTTLE_SPEED_TPS : speed_tps;
    throttle_val_ = interpolate_(speed_tps) / (float)1e6 * THROTTLE_PWM_FREQUENCY_HZ * (1<<PWM_RESOLUTION_BITS);
    analogWrite(THROTTLE_PIN, throttle_val_);
}

int Vehicle::interpolate_(float speed_tps) {
    int lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS;
    int lower_idx = (lower_val - MIN_THROTTLE_SPEED_TPS)/LOOKUP_STEP_TPS;
    int val = lower_val + (speed_tps - lower_val)*(lookup[lower_idx+1]-lower_val)/20;
    if (val<UPPER_DEADBAND || val>LOWER_DEADBAND) { return 1500; }
    else { return val; }
}
