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
#define VEHICLE_GEAR_RATIO        5.62          // Gear ratio (motor revs per wheel rev).
#define VEHICLE_WHEEL_RAD_MM     32.385         // Wheel diameter in millimeters.
#define MAX_STEERING_ANGLE_RAD  (M_PI/6)        // Max one-sided steering servo angle.
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

// Convert between m/s and tic/s
#define MPS_TO_TPS(mps) ((mps)*1000.0 / (2*M_PI*VEHICLE_WHEEL_RAD_MM) * VEHICLE_GEAR_RATIO * 6)
#define TPS_TO_MPS(tps) ((tps)/ VEHICLE_GEAR_RATIO / 6 * 2*M_PI*VEHICLE_WHEEL_RAD_MM / 1000 )

// Convert from steering angle [rad] to pulse width [us]
#define STEERING_RAD_TO_PW_US(rad) (409.72*rad + 1500)

// Speed PID parameters
#define KP   0.03
//#define KI   0.0001
#define KI   0
#define KD   0.005
#define ERR_INT_LIMIT 2000

void estop_isr(void);
void odom_isr(void);
        
class VehicleDef {
    public:
        bool estop_status;

        double velocity_measured_tps;
        double velocity_commanded_tps;
        double velocity_setpoint_tps;

        int64_t odom;
        IntervalTimer odom_timer;

        VehicleDef();
        ~VehicleDef();

        void set_steering_angle(double) volatile;
        void set_velocity_setpoint(double) volatile;
        double get_odom_m(void) volatile;
        double get_speed_tps(void) volatile;
        double get_speed_mps(void) volatile;
        double get_steering_rad(void) volatile { return steering_ang_rad; }
        int get_velocity_pwm(void) volatile { return velocity_pwm_val; }
        int get_steering_pwm(void) volatile { return steering_pwm_val; }
        void send_speed(void) volatile;
        void update_pid(void) volatile;

        double pub_err;
        double pub_err_int;
        double pub_err_d;

    private:
        uint32_t steering_pwm_val;
        uint32_t velocity_pwm_val;



        double steering_ang_rad;
        int interpolate_(double) volatile;
};

volatile extern VehicleDef Vehicle;

#endif
