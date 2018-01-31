#include "vehicle.h"

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <teensy/Feedback.h>
#include <teensy/Command.h>

void commandCallback(const teensy::Command& cmd) {
    // Set throttle speed and steering angle according to command.
    Vehicle.set_velocity_setpoint(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);

    // Set estop flag if commanded by Jetson.
    if (cmd.estop) { estop_isr(); }
}

int main(void) {

    ros::NodeHandle nh;
    nh.initNode();

    teensy::Feedback fdb;
    ros::Publisher pub("/teensy/feedback", &fdb);
    nh.advertise(pub);

    ros::Subscriber<teensy::Command> sub("/teensy/command", &commandCallback);
    nh.subscribe(sub);

    while (!Vehicle.estop_status) {
        fdb.odom_m = Vehicle.get_odom_m();
        fdb.vel_mps = Vehicle.get_speed_mps();
        fdb.setpoint  = TPS_TO_MPS(Vehicle.velocity_setpoint_tps);
        fdb.measured  = TPS_TO_MPS(Vehicle.velocity_measured_tps);
        fdb.command  = TPS_TO_MPS(Vehicle.velocity_commanded_tps);
        fdb.err = Vehicle.pub_err;
        fdb.i_err = Vehicle.pub_err_int;
        fdb.d_err = Vehicle.pub_err_d;

        pub.publish(&fdb);
        nh.spinOnce();
        delay(5);
        Vehicle.update_pid();
    }

    fdb.estop = Vehicle.estop_status;
    pub.publish(&fdb);

    return 0;
}
