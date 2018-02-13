#ifndef TEENSY_VEHICLE_H_
#define TEENSY_VEHICLE_H_

#include <Arduino.h>    // IntervalTimer
#include <cstdint>      // fixed width types

// Control board pinout
#define STEERING_PIN    4
#define THROTTLE_PIN   23
#define LED_PIN        13
#define ESTOP_PIN       2

// Motor hall effect pins
#define MOTOR_PHASE0_PIN 3
#define MOTOR_PHASE1_PIN 6
#define MOTOR_PHASE2_PIN 5

// Vehicle mechanical constants 
#define VEHICLE_GEAR_RATIO       5.62           // Gear ratio (motor revs per wheel rev).
#define VEHICLE_WHEEL_RAD_MM   32.385           // Wheel diameter in millimeters.
#define MAX_STEERING_ANGLE_RAD  (M_PI*0.25)     // Max one-sided steering excursion at the wheels.
#define WHEELBASE_FRONT_MM        67.09
#define WHEELBASE_REAR_MM        189.91

// PWM
#define PWM_RESOLUTION_BITS            16       // 
#define THROTTLE_PWM_FREQUENCY_HZ     480       // The ESC accepts pulses at up to 480 Hz.
#define STEERING_PWM_FREQUENCY_HZ      76       // The servo accepts pulses at 76 Hz.

// Throttle range
#define THROTTLE_FWD_MAX  ((1<<PWM_RESOLUTION_BITS)*1.00*THROTTLE_PWM_FREQUENCY_HZ/1000.0)  // Send to analogWrite() for maximum forward speed.
#define THROTTLE_STOP ((1<<PWM_RESOLUTION_BITS)*1.50*THROTTLE_PWM_FREQUENCY_HZ/1000.0)      // Send to analogWrite() for zero speed.
#define THROTTLE_REV_MAX  ((1<<PWM_RESOLUTION_BITS)*2.00*THROTTLE_PWM_FREQUENCY_HZ/1000.0)  // Send to analogWrite() for maximum reverse speed.

// Steering range
#define STEERING_LEFT_MAX   ((1<<PWM_RESOLUTION_BITS)*1.75*STEERING_PWM_FREQUENCY_HZ/1000.0)    // Send to analogWrite() for maximum left turn.
#define STEERING_CENTER     ((1<<PWM_RESOLUTION_BITS)*1.5*STEERING_PWM_FREQUENCY_HZ/1000.0)     // Send to analogWrite() to go straight.
#define STEERING_RIGHT_MAX  ((1<<PWM_RESOLUTION_BITS)*1.25*STEERING_PWM_FREQUENCY_HZ/1000.0)    // Send to analogWrite() for maximum right turn.

// Speed PID parameters
#define KP   0.3
#define KI   0
#define KD   0


        
constexpr uint8_t hall_effect_sequence[6] = {0, 2, 1, 4, 5, 3};

class VehicleDef {
    public:
        bool estop_status;
        uint32_t steering_val;
        uint32_t throttle_val;
        double sent_speed_tps;
        double actual_speed_tps;
        double target_speed_tps;
        double steering_ang_rad;
        uint64_t odom;
        IntervalTimer odom_timer;

        VehicleDef();
        ~VehicleDef();

        void set_steering_angle(double) volatile;
        void set_throttle_speed(double) volatile;
        void run_profile(int) volatile;
        double get_odom_m(void) volatile;
        double get_speed_tps(void) volatile;
        double get_speed_mps(void) volatile;
        double get_steering_rad(void) volatile { return steering_ang_rad; }
        int get_throttle_val(void) volatile { return throttle_val; }
        int get_steering_val(void) volatile { return steering_val; }
        void send_speed(void) volatile;

    private:
        int interpolate_(double) volatile;
};

volatile extern VehicleDef Vehicle;

#endif
