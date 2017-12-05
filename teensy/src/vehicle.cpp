// Copyright 2017 Jordan Ford

#include "vehicle.h"

// Global static pointer used to ensure a single instance of the class.
Vehicle* Vehicle::pInstance = NULL;

void Vehicle::Instantiate(void) {
    if (!pInstance)
        pInstance = new Vehicle();
}

Vehicle* Vehicle::Instance(void) {
    return pInstance;
}

Vehicle::Vehicle(void) : steering_servo_period_(0),
                         throttle_servo_period_(0) {

   pinMode(LED_PIN, OUTPUT);
   digitalWriteFast(13, HIGH);

   pinMode(ESTOP_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, HIGH);

   accel_ = Adafruit_BNO055();
   if (!accel_.begin()) {
       error_blink(INIT_ACCEL_ERR_PERIOD);
   }
   delay(1000);
   accel_.setExtCrystalUse(true);

   steering_servo_.attach(STEERING_PIN);
   throttle_servo_.attach(THROTTLE_PIN);

   set_steering_angle(0);
   set_throttle_speed(0);
}

Vehicle::~Vehicle(void) {}

void Vehicle::set_steering_angle(float angle_rad) {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;

    // Obtain a value between 0 and 1.
    steering_servo_period_ = 1 + angle_rad/(2*MAX_STEERING_ANGLE_RAD);
    // Obtain a value between 0 and STEER_RANGE.
    steering_servo_period_ = steering_servo_period_*(STEER_LEFT_PERIOD-STEER_RIGHT_PERIOD);
    // Obtain a value between STEER_RIGHT_PERIOD and STEER_LEFT_PERIOD.
    steering_servo_period_ = steering_servo_period_ + STEER_RIGHT_PERIOD;
    // Write the new period to the steering servo.
    steering_servo_.write(steering_servo_period_);
}

void Vehicle::set_throttle_speed(float speed_mps) {
    throttle_servo_.write(speed_mps);
}

void Vehicle::error_blink(int period_ms) {
    while(1) {
        digitalWriteFast(LED_PIN, LOW);
        delay(period_ms>>1);
        digitalWriteFast(LED_PIN, HIGH);
        delay(period_ms>>1);
    }
}

void Vehicle::estop_isr(void) {
    pInstance->set_throttle_speed(0);
    pInstance->error_blink(ESTOP_ERR_PERIOD);
}
