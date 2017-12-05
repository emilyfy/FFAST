#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "BNO055/BNO055.h"
#include "PWMServo/PWMServo.h"
#include <Arduino.h>

// Error blink periods
#define ESTOP_ERR_PERIOD                1000
#define INIT_ACCEL_ERR_PERIOD           2000
#define VEHICLE_SINGLETON_ERR_PERIOD    3000

// Control board pinout
#define STEERING_PIN    4
#define THROTTLE_PIN    3
#define LED_PIN        13
#define ESTOP_PIN       5

// Servo control input limits
#define STEER_LEFT_PERIOD       1.75
#define STEER_RIGHT_PERIOD      1.25
#define FULL_SPEED_FWD_PERIOD   1.00
#define FULL_SPEED_REV_PERIOD   2.00

// Vehicle max steering angle at the wheels.
#define MAX_STEERING_ANGLE_RAD  (M_PI*0.25)


class Vehicle {
    public:
        static void     Instantiate();
        static Vehicle*    Instance();

        void set_steering_angle(float angle_rad);
        void set_throttle_speed(float speed_mps);

    private:
        Vehicle();                  // constructor cannot be called.
        Vehicle(Vehicle const&){};  // copy constructor is private.
        ~Vehicle();                 // destructor cannot be called.
        static Vehicle* pInstance;

    private:
        Adafruit_BNO055 accel_;

        PWMServo steering_servo_;
        PWMServo throttle_servo_;
        float steering_servo_period_;
        float throttle_servo_period_;

        void error_blink(int period_ms);
        static void estop_isr(void);
};

#endif
