// Copyright 2017 Jordan Ford

#include "vehicle.h"
#include "lookup.h"
#include <Arduino.h>

/********** Global Variables **********/
VehicleDef Vehicle;

/********** E-Stop Interrupt Routine **********/
void estop_isr(void) {
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    analogWrite(STEERING_PIN, STEERING_CENTER);
    Serial.write('e');
    Serial.write('\n');
}

/********** Hall Effect Sensors Interrupt Routine **********/
void odom_isr() {
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
        if (curr_time - last_time > 19000) { Vehicle.actual_speed_tps = 0.0; }
        else { return; }
    }
    else if (state-1 == prev_state || (state == 0 && prev_state == 5)) { 
        Vehicle.odom++;
        Vehicle.actual_speed_tps = (float)1e6/(curr_time - last_time);
    }
    else if (state+1 == prev_state || (state == 5 && prev_state == 0)) { 
        Vehicle.odom--;
        Vehicle.actual_speed_tps = (float)-1e6/(curr_time - last_time);
    }
    else { estop_isr(); }

    // PID on throttle value
    float err = Vehicle.target_speed_tps - Vehicle.actual_speed_tps;
    static float err_prev = err;
    static float err_int = 0;

    Vehicle.sent_speed_tps -= KP*err + KI*err_int + KD*(err-err_prev);
    Vehicle.send_speed();

    // Update
    err_int = 0.9*err_int + 0.1*err;
    err_prev = err;
    last_time = curr_time;
    prev_state = state;
}

/********** Constructor **********/
VehicleDef::VehicleDef(void) : estop_status(false),
                               steering_val(STEERING_CENTER),
                               throttle_val(THROTTLE_STOP),
                               sent_speed_tps(0),
                               actual_speed_tps(0),
                               target_speed_tps(0),
                               odom(0) {
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
    //attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
    
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
VehicleDef::~VehicleDef(void) {}


/********** Public Functions : Setters **********/
void VehicleDef::set_steering_angle(float angle_rad) {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;
    steering_val = map( angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD, STEERING_RIGHT_MAX, STEERING_LEFT_MAX );
    analogWrite(STEERING_PIN, steering_val);
}

void VehicleDef::set_throttle_speed(float speed_mps) {
    Vehicle.target_speed_tps = speed_mps * 1000.0 / (2*M_PI*VEHICLE_WHEEL_RAD_MM) * VEHICLE_GEAR_RATIO * 6;
    sent_speed_tps = Vehicle.target_speed_tps;
    Vehicle.send_speed();
}

void VehicleDef::run_profile(int profile) {
}

/********** Public Functions : Getters **********/
double VehicleDef::get_odom_m(void) {
    return ( Vehicle.odom / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ; 
}

double VehicleDef::get_speed_tps(void) {
    return actual_speed_tps;
}

double VehicleDef::get_speed_mps(void) {
    return ( actual_speed_tps / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ; 
}

/********** Private Functions **********/
void VehicleDef::send_speed(void) {
    // Cap speed at the vehicle limits.
    sent_speed_tps = (sent_speed_tps > MAX_THROTTLE_SPEED_TPS) ? MAX_THROTTLE_SPEED_TPS : sent_speed_tps;
    sent_speed_tps = (sent_speed_tps < MIN_THROTTLE_SPEED_TPS) ? MIN_THROTTLE_SPEED_TPS : sent_speed_tps;
    throttle_val = interpolate_(sent_speed_tps) / (float)1e6 * THROTTLE_PWM_FREQUENCY_HZ * (1<<PWM_RESOLUTION_BITS);
    analogWrite(THROTTLE_PIN, throttle_val);
}

int VehicleDef::interpolate_(float speed_tps) {
    int lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS;
    int lower_idx = (lower_val - MIN_THROTTLE_SPEED_TPS)/LOOKUP_STEP_TPS;
    int val = lower_val + (speed_tps - lower_val)*(lookup[lower_idx+1]-lower_val)/20;
    if (val<UPPER_DEADBAND || val>LOWER_DEADBAND) { return 1500; }
    else { return val; }
}
