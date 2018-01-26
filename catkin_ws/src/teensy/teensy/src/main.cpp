#include "vehicle.h"

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <teensy/Feedback.h>
#include <teensy/Command.h>

void commandCallback(const teensy::Command& cmd) {
    // Set throttle speed and steering angle according to command.
    Vehicle.set_throttle_speed(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);

    // Set estop flag if commanded by Jetson.
    if (cmd.estop) { Vehicle.estop_status = true; }

    // Run a profile if prompted to. Default value of 0 means no profile selected.
    if (cmd.profile) { Vehicle.run_profile(cmd.profile); }
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
        pub.publish(&fdb);
        nh.spinOnce();
        delay(5);
    }

    fdb.estop = Vehicle.estop_status;
    pub.publish(&fdb);

    return 0;
}
