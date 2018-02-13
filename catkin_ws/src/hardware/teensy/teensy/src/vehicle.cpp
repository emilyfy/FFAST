// Copyright 2017 Jordan Ford

#include "vehicle.h"
#include "lookup.h"
#include <Arduino.h>

// Periodically call the odom_isr to update
// the velocity in case the motor has stopped moving.
#define ODOM_ISR_TIMER_DELAY_US  20000
#define SPEED_FILTER_TAU_US      100000

/********** Global Variables **********/
volatile VehicleDef Vehicle;

/********** E-Stop Interrupt Routine **********/
void estop_isr(void) {
    // Center the steering.
    analogWrite(STEERING_PIN, STEERING_CENTER);
    // Kill the throttle.
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    // Set a flag so the rest of the world will know the estop was triggered.
    Vehicle.estop_status = true;
}

/********** Hall Effect Sensors Interrupt Routine **********/
void odom_isr() {
    static double instantaneous_speed_tps = 0;
    static int prev_state = -1;
    uint32_t curr_time = micros();
    static uint32_t last_time = curr_time;

    // When the motor spins in the direction that moves the
    // the vehicle forward, its hall effect sensors [P0, P1, P2]
    // follow this sequence:
    //
    //  Index | P2 | P1 | P0 | Binary Motor State
    // -------|----|----|----|--------------------
    //      0 |  0 |  0 |  1 |                  1
    //      1 |  0 |  1 |  1 |                  3
    //      2 |  0 |  1 |  0 |                  2
    //      3 |  1 |  1 |  1 |                  6
    //      4 |  1 |  0 |  0 |                  4
    //      5 |  1 |  0 |  1 |                  5
    //
    // Converting [P2, P1, P0] to binary yields the Binary Motor State (BMS).
    uint8_t binary_motor_state = (digitalRead(MOTOR_PHASE0_PIN) << 2 |
                                  digitalRead(MOTOR_PHASE1_PIN) << 1 |
                                  digitalRead(MOTOR_PHASE2_PIN));

    // In order to decide whether the robot has moved forward or backward,
    // we need to convert the BMS to its index. Because the indices count from 0 to 5,
    // a positive change in the index indicates forward vehicle movement and a negative
    // change indicates backward movement.
    //
    // To convert BMS to index, look up BMS-1 in the following lookup table.
    constexpr uint8_t bms2ind[] = {0, 2, 1, 4, 5, 3};

    // The resulting value is the index of the current motor state.
    int state = bms2ind[binary_motor_state-1];

    // The first time the odom_isr runs, it will ignore the tic.
    // TODO: Fix this?
    if (prev_state < 0) {
        prev_state = state;
        return;
    }

    // If the isr triggered, but it wasn't caused by a toggling hall effect sensor,
    // it must have been triggered by the timer.
    if (prev_state == state) {
        if (curr_time - last_time > .95*ODOM_ISR_TIMER_DELAY_US) {
            instantaneous_speed_tps = 0.0;
        }
        else { return; }
    }
    // If the state has increased, add to odom, and update the speed.
    else if (state == prev_state+1 || (state == 0 && prev_state == sizeof(bms2ind)-1)) {
        Vehicle.odom++;
        instantaneous_speed_tps = (float)1e6/(curr_time - last_time);
    }
    // If the state has increased, subtract from odom, and update the speed.
    else if (state == prev_state-1 || (state == sizeof(bms2ind)-1 && prev_state == 0)) {
        Vehicle.odom--;
        instantaneous_speed_tps = (float)-1e6/(curr_time - last_time);
    }
    // Something has gone wrong.
    // Most likely we missed a tic due to interrupt latency.
    // TODO: Is there a better way to handle this?
    else { estop_isr(); return; }

    // Exponential moving average coefficient for speed measurement
    double speed_filter_alpha = 1 - exp( - (double)(curr_time - last_time) / (double)SPEED_FILTER_TAU_US );

    // Filter the actual speed with an exponential moving average
    Vehicle.velocity_measured_tps += speed_filter_alpha*(instantaneous_speed_tps-Vehicle.velocity_measured_tps);
    
    // Track the last time the isr was called.
    last_time = curr_time;

    // Track the motor state so we can see when and how it changes.
    prev_state = state;
}

/********** Constructor **********/
VehicleDef::VehicleDef(void) : estop_status(false),
                               velocity_measured_tps(0),
                               velocity_commanded_tps(0),
                               velocity_setpoint_tps(0),
                               odom(0),
                               steering_pwm_val(STEERING_CENTER),
                               velocity_pwm_val(THROTTLE_STOP),
                               steering_ang_rad(0) {
    // Set up the PWM.
    analogWriteResolution(PWM_RESOLUTION_BITS);
    analogWriteFrequency(THROTTLE_PIN, THROTTLE_PWM_FREQUENCY_HZ);
    analogWriteFrequency(STEERING_PIN, STEERING_PWM_FREQUENCY_HZ);

    // Set the steering to the center and the throttle to off.
    analogWrite(STEERING_PIN, STEERING_CENTER);
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);

    // Initialize the LED to OFF.
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Attach the ESTOP service routine.
    pinMode(ESTOP_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);

    // Attach the odom_isr to the motor hall effect sensors.
    pinMode(MOTOR_PHASE0_PIN, INPUT);
    pinMode(MOTOR_PHASE1_PIN, INPUT);
    pinMode(MOTOR_PHASE2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE0_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE1_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE2_PIN), odom_isr, CHANGE);

    // Attach the odom_isr to a 20ms timer so it can detect when motor velocity is zero.
    // Without this, the isr won't trigger when the car isn't moving, and it will retain
    // whatever speed was calculated right before the speed went to zero.
    odom_timer.begin(odom_isr, ODOM_ISR_TIMER_DELAY_US);
}

/********** Destructor **********/
VehicleDef::~VehicleDef(void) {
    // Detach all interrupts
    detachInterrupt(digitalPinToInterrupt(ESTOP_PIN));
    detachInterrupt(digitalPinToInterrupt(MOTOR_PHASE0_PIN));
    detachInterrupt(digitalPinToInterrupt(MOTOR_PHASE1_PIN));
    detachInterrupt(digitalPinToInterrupt(MOTOR_PHASE2_PIN));
}


/********** Public Functions **********/
void VehicleDef::set_steering_angle(double angle_rad) volatile {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;

    // Update global steering angle value
    steering_ang_rad = angle_rad;

    // Map from angle to PWM value according to equation obtained from experiment and send to servo
    steering_pwm_val = STEERING_RAD_TO_PW_US(angle_rad) / (float)1e6 * STEERING_PWM_FREQUENCY_HZ * (1<<PWM_RESOLUTION_BITS);
    if (!estop_status) { analogWrite(STEERING_PIN, steering_pwm_val); }
}

void VehicleDef::set_velocity_setpoint(double new_setpoint_mps) volatile {
    // Convert value from m/s to tic/s and set it as target value
    velocity_setpoint_tps = MPS_TO_TPS(new_setpoint_mps);
}

/********** Function to send speed to motor **********/
void VehicleDef::send_speed(void) volatile {
    // Cap speed at the vehicle limits.
    velocity_commanded_tps = (velocity_commanded_tps > MAX_THROTTLE_SPEED_TPS) ? MAX_THROTTLE_SPEED_TPS : velocity_commanded_tps;
    velocity_commanded_tps = (velocity_commanded_tps < MIN_THROTTLE_SPEED_TPS) ? MIN_THROTTLE_SPEED_TPS : velocity_commanded_tps;
    velocity_pwm_val = interpolate_(velocity_commanded_tps) / (float)1e6 * THROTTLE_PWM_FREQUENCY_HZ * (1<<PWM_RESOLUTION_BITS);
    if (!estop_status) { analogWrite(THROTTLE_PIN, velocity_pwm_val); }
}

/********** Update PID Function **********/
void VehicleDef::update_pid(void) volatile {
    double err = velocity_setpoint_tps - velocity_measured_tps;
    static double err_prev = err;
    static double err_int = 0;
    uint32_t curr_time = millis();
    static uint32_t last_time = curr_time;
    double dt = (double)(curr_time - last_time)/1e3;

    velocity_commanded_tps += KP*err + -KI*err_int + ((curr_time != last_time) ? KD*(err-err_prev)/dt : 0.0);
    send_speed();

    pub_err = err;
    pub_err_int = err_int;
    pub_err_d = (err-err_prev)/dt;

    // Update integral and previous
    err_int += err*dt;
    if (err_int > ERR_INT_LIMIT) { err_int = ERR_INT_LIMIT; }
    else if (err_int < -ERR_INT_LIMIT) { err_int = -ERR_INT_LIMIT; }
    err_prev = err;
    last_time = curr_time;
}

/********** Public Functions : Getters **********/
double VehicleDef::get_odom_m(void) volatile {
    return ( odom / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ;
}

double VehicleDef::get_speed_tps(void) volatile {
    return velocity_measured_tps;
}

double VehicleDef::get_speed_mps(void) volatile {
    return TPS_TO_MPS(velocity_measured_tps);
}

/********** Private Function: linear interpolation on lookup array **********/
int VehicleDef::interpolate_(double speed_tps) volatile {
    int lower_val;

    // Calculate the lower value of speed that is divisible by LOOKUP_STEP_TPS
    // Perform integer division then multiply by the step, subtract the step if it's a negative value
    // Get the index of the lower value
    if (speed_tps>0) { lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS; }
    else { lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS - LOOKUP_STEP_TPS; }
    int lower_idx = (lower_val - MIN_THROTTLE_SPEED_TPS)/LOOKUP_STEP_TPS;

    // Perform linear interpolation
    int val = lookup[lower_idx] + (speed_tps - lower_val)*(lookup[lower_idx+1]-lookup[lower_idx])/20;

    // Send 1500 if value inside deadband
    if (val<=MID_UPPER_DEADBAND && val>=MID_LOWER_DEADBAND) { return 1500; }
    else if (val<UPPER_DEADBAND && val>MID_UPPER_DEADBAND) { return UPPER_DEADBAND; }
    else if (val>LOWER_DEADBAND && val<MID_LOWER_DEADBAND) { return LOWER_DEADBAND; }
    else { return val; }
}
