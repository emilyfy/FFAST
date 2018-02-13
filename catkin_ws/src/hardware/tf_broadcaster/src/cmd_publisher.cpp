#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <teensy_msgs/Command.h>

#define WHEELBASE_M 0.257

ros::Publisher cmd_pub;

double ang_vel_to_steering(double v, double omega) {
    double steering_ang_rad;
    if (v == 0 || omega == 0) {
        steering_ang_rad = 0;
    }
    else {
        double turn_radius = v / omega;
        steering_ang_rad = atan(WHEELBASE_M / turn_radius);
    }
    return steering_ang_rad;
}

void cmdvelCallback(const geometry_msgs::Twist& twist)
{
    teensy_msgs::Command cmd;

    cmd.vel_mps = twist.linear.x;
    cmd.steering_rad = ang_vel_to_steering(twist.linear.x, twist.angular.z);
    cmd_pub.publish(cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_publisher");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<teensy_msgs::Command>("/teensy/command", 10);
    ros::Subscriber cmdvel_sub = nh.subscribe("/cmd_vel",10,cmdvelCallback);

    ros::spin();
    
    return 0;
}
