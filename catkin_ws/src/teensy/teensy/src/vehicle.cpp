// Copyright 2017 Jordan Ford

#include "vehicle.h"
#include "lookup.h"
#include <Arduino.h>

// Periodically call the odom_isr to update
// the velocity in case the motor has stopped moving.
#define ODOM_ISR_TIMER_DELAY_US 20000

/********** Global Variables **********/
volatile VehicleDef Vehicle;

/********** E-Stop Interrupt Routine **********/
void estop_isr(void) {
    // Kill the throttle.
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    // Center the steering.
    analogWrite(STEERING_PIN, STEERING_CENTER);
    // Set a flag so the rest of the world will know the estop was triggered.
    Vehicle.estop_status = true;
}

/********** Hall Effect Sensors Interrupt Routine **********/
void odom_isr() {
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
        if (curr_time - last_time > ODOM_ISR_TIMER_DELAY_US) { Vehicle.actual_speed_tps = 0.0; }
        else { return; }
    }
    // If the state has increased, add to odom, and update the speed.
    else if (state == prev_state+1 || (state == 0 && prev_state == sizeof(bms2ind)-1)) {
        Vehicle.odom++;
        Vehicle.actual_speed_tps = (float)1e6/(curr_time - last_time);
    }
    // If the state has increased, subtract from odom, and update the speed.
    else if (state == prev_state-1 || (state == sizeof(bms2ind)-1 && prev_state == 0)) {
        Vehicle.odom--;
        Vehicle.actual_speed_tps = (float)-1e6/(curr_time - last_time);
    }
    // Something has gone wrong.
    // Most likely we missed a tic due to interrupt latency.
    // TODO: Is there a better way to handle this?
    else { estop_isr(); }

    // Track the last time the isr was called.
    last_time = curr_time;

    // Track the motor state so we can see when and how it changes.
    prev_state = state;
}

/********** Constructor **********/
VehicleDef::VehicleDef(void) : estop_status(false),
                               actual_speed_tps(0),
                               odom(0),
                               steering_val(STEERING_CENTER),
                               throttle_val(THROTTLE_STOP),
                               sent_speed_tps(0),
                               target_speed_tps(0),
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
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);

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


/********** Public Functions : Setters **********/
void VehicleDef::set_steering_angle(double angle_rad) volatile {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;

    // Update global steering angle value
    steering_ang_rad = angle_rad;

    // Linear mapping from angle to PWM value. May need to test this out and see if it's accurate. Create another lookup table if not
    steering_val = map( angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD, STEERING_RIGHT_MAX, STEERING_LEFT_MAX );

    // Write PWM value to servo
    if (!estop_status) { analogWrite(STEERING_PIN, steering_val); }
}

void VehicleDef::set_throttle_speed(double speed_mps) volatile {
    // Convert value from m/s to tic/s and set it as target value
    Vehicle.target_speed_tps = speed_mps * 1000.0 / (2*M_PI*VEHICLE_WHEEL_RAD_MM) * VEHICLE_GEAR_RATIO * 6;

    // Call to function to send speed as initial estimate
    Vehicle.send_speed(target_speed_tps);
}

void VehicleDef::run_profile(int profile) volatile {
}

/********** Public Functions : Getters **********/
double VehicleDef::get_odom_m(void) volatile {
    return ( Vehicle.odom / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ;
}

double VehicleDef::get_speed_tps(void) volatile {
    return actual_speed_tps;
}

double VehicleDef::get_speed_mps(void) volatile {
    return ( actual_speed_tps / VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 ) ;
}

/********** Update PID **********/
void VehicleDef::update_pid(void) volatile {

}

/********** Function to send speed to motor **********/
void VehicleDef::send_speed(double sent_speed_tps) volatile {
    // Cap speed at the vehicle limits.
    sent_speed_tps = (sent_speed_tps > MAX_THROTTLE_SPEED_TPS) ? MAX_THROTTLE_SPEED_TPS : sent_speed_tps;
    sent_speed_tps = (sent_speed_tps < MIN_THROTTLE_SPEED_TPS) ? MIN_THROTTLE_SPEED_TPS : sent_speed_tps;
    throttle_val = interpolate_(sent_speed_tps) / (float)1e6 * THROTTLE_PWM_FREQUENCY_HZ * (1<<PWM_RESOLUTION_BITS);
    if (!estop_status) { analogWrite(THROTTLE_PIN, throttle_val); }
}

/********** Private Functions **********/
int VehicleDef::interpolate_(double speed_tps) volatile {
    int lower_val;

    // Calculate the lower value of speed that is divisible by LOOKUP_STEP_TPS and get the index
    if (speed_tps>0) { lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS; }
    else { lower_val = (int)speed_tps/LOOKUP_STEP_TPS*LOOKUP_STEP_TPS - LOOKUP_STEP_TPS; }
    int lower_idx = (lower_val - MIN_THROTTLE_SPEED_TPS)/LOOKUP_STEP_TPS;

    // Perform linear interpolation
    int val = lookup[lower_idx] + (speed_tps - lower_val)*(lookup[lower_idx+1]-lookup[lower_idx])/20;

    // Send 1500 if value inside deadband
    if (val<UPPER_DEADBAND && val>LOWER_DEADBAND) { return lookup[(0-MIN_THROTTLE_SPEED_TPS)/LOOKUP_STEP_TPS]; }
    else { return val; }
}
