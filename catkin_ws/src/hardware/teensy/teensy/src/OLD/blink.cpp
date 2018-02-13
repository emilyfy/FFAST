#include <Arduino.h>
#include "vehicle.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <teensy/Command.h>
#include <teensy/Feedback.h>
#include <ros.h>
#include <ros/time.h>

ros::NodeHandle nh;
teensy::Feedback fdb;
ros::Publisher pub("/Teensy/feedback", &fdb);

void commandCallback(const teensy::Command& cmd) {
    digitalWrite(13, !digitalRead(13));
    Vehicle.set_throttle_speed(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);
    if (cmd.estop) { Vehicle.estop_status = true; }
    if (cmd.profile) { Vehicle.run_profile(cmd.profile); }
}
ros::Subscriber<teensy::Command> sub("/Teensy/command", &commandCallback);

int main(void) {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
    while(1) {
        nh.now();
        nh.spinOnce();
        fdb.odom_m = Vehicle.get_odom_m();
        fdb.vel_mps = Vehicle.get_speed_mps();
        pub.publish(&fdb);
        delay(5);
    }
    return 0;
}
