#include "vehicle.h"
#include <Arduino.h>
#include <ros.h>
#include "teensy/Command.h"
#include "teensy/Feedback.h"

ros::NodeHandle nh;

teensy::Feedback fdb;
ros::Publisher pub("/Teensy/feedback", &fdb);

void commandCallback(const teensy::Command& cmd) {
    Vehicle.set_throttle_speed(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);
    if (cmd.estop) { Vehicle.estop_status = true; }
    if (cmd.profile) { Vehicle.run_profile(cmd.profile); }
}
ros::Subscriber<teensy::Command> sub("/Teensy/command", &commandCallback);

int main(int argc, char** argv){
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);

    fdb.estop = false;
    
    while (1) {

        nh.spinOnce();

        fdb.odom_m = Vehicle.get_odom_m();
        fdb.vel_mps = Vehicle.get_speed_mps();
        pub.publish(&fdb);
        delay(1000);

    }

    return 0;
}
