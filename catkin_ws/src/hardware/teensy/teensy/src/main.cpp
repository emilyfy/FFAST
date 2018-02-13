#include "vehicle.h"

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <teensy_msgs/Feedback.h>
#include <teensy_msgs/Command.h>
#include <teensy_msgs/Debug.h>

void commandCallback(const teensy_msgs::Command& cmd) {
    // Set throttle speed and steering angle according to command.
    Vehicle.set_velocity_setpoint(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);

    // Set estop flag if commanded by Jetson.
    if (cmd.estop) { estop_isr(); }
}

int main(void) {

    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    teensy_msgs::Feedback fdb;
    ros::Publisher pub_fdb("/teensy/feedback", &fdb);
    nh.advertise(pub_fdb);

    teensy_msgs::Debug debug_msg;
    ros::Publisher pub_debug("/teensy/debug", &debug_msg);
    nh.advertise(pub_debug);

    ros::Subscriber<teensy_msgs::Command> sub("/teensy/command", &commandCallback);
    nh.subscribe(sub);

    while (!Vehicle.estop_status) {
        fdb.odom_m = Vehicle.get_odom_m();
        fdb.vel_mps = Vehicle.get_speed_mps();
        debug_msg.setpoint  = TPS_TO_MPS(Vehicle.velocity_setpoint_tps);
        debug_msg.measured  = TPS_TO_MPS(Vehicle.velocity_measured_tps);
        debug_msg.command  = TPS_TO_MPS(Vehicle.velocity_commanded_tps);
        debug_msg.pwm = Vehicle.get_velocity_pwm()*(float)1e6/THROTTLE_PWM_FREQUENCY_HZ/(1<<PWM_RESOLUTION_BITS);
        debug_msg.err = Vehicle.pub_err;
        debug_msg.i_err = Vehicle.pub_err_int;
        debug_msg.d_err = Vehicle.pub_err_d;

        pub_fdb.publish(&fdb);
        pub_debug.publish(&debug_msg);
        nh.spinOnce();
        delay(5);
        Vehicle.update_pid();
    }

    fdb.estop = Vehicle.estop_status;
    fdb.vel_mps = 0;
    pub.publish(&fdb);
    nh.logerror("Estop on vehicle triggered.");

    return 0;
}
