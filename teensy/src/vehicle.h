#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <Arduino.h>

// Control board pinout
#define STEERING_PIN    4
#define THROTTLE_PIN   23
#define LED_PIN        13
#define ESTOP_PIN       2

// Motor hall effect pins
#define MOTOR_PHASE0_PIN 3
#define MOTOR_PHASE1_PIN 6
#define MOTOR_PHASE2_PIN 5

// Vehicle Mechanical Constants 
#define VEHICLE_GEAR_RATIO       5.62
#define VEHICLE_WHEEL_RAD_MM   32.385

// PWM
#define PWM_RESOLUTION_BITS            16
#define THROTTLE_PWM_FREQUENCY_HZ     480
#define STEERING_PWM_FREQUENCY_HZ      76

// Throttle Range
#define THROTTLE_FWD_MAX  ((1<<PWM_RESOLUTION_BITS)*1.00*THROTTLE_PWM_FREQUENCY_HZ/1000.0)
#define THROTTLE_STOP ((1<<PWM_RESOLUTION_BITS)*1.50*THROTTLE_PWM_FREQUENCY_HZ/1000.0)
#define THROTTLE_REV_MAX  ((1<<PWM_RESOLUTION_BITS)*2.00*THROTTLE_PWM_FREQUENCY_HZ/1000.0)

// Steering Range
#define STEERING_LEFT_MAX   ((1<<PWM_RESOLUTION_BITS)*1.75*STEERING_PWM_FREQUENCY_HZ/1000.0)
#define STEERING_CENTER     ((1<<PWM_RESOLUTION_BITS)*1.5*STEERING_PWM_FREQUENCY_HZ/1000.0)
#define STEERING_RIGHT_MAX  ((1<<PWM_RESOLUTION_BITS)*1.25*STEERING_PWM_FREQUENCY_HZ/1000.0)

// Speed PID Parameters
#define KP   0.3
#define KI   0
#define KD   0

// Error blink periods
#define ESTOP_ERR_PERIOD                1000
#define INIT_ACCEL_ERR_PERIOD           2000
#define VEHICLE_SINGLETON_ERR_PERIOD    3000

// Vehicle max steering angle at the wheels
#define MAX_STEERING_ANGLE_RAD  (M_PI*0.25)

constexpr uint8_t hall_effect_sequence[6] = {0, 2, 1, 4, 5, 3};
extern volatile long long odom;
extern volatile double target_speed_tps_;
extern volatile double actual_speed_tps_;
extern volatile double sent_speed_tps_;
extern volatile uint16_t throttle_val_;
extern IntervalTimer odom_timer;
        
class Vehicle {
    public:
        bool estop_status = false;

        Vehicle();
        ~Vehicle();

        void set_steering_angle(float);
        void set_throttle_speed(float);
        void run_profile(int);
        double get_odom_m(void);
        double get_speed_tps(void);
        double get_speed_mps(void);
        static void odom_isr(void);
        static void estop_isr(void);
        static void error_blink(int);
        int get_throttle_val(void) { return throttle_val_; }
        int get_steering_val(void) { return steering_val_; }

    private:
        uint16_t steering_val_;
        static void send_speed_(float);
        static int interpolate_(float);
};

#endif
