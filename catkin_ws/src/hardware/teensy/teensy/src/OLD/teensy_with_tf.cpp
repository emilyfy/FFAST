#include "vehicle.h"

#include "teensy/Command.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <ros.h>
#include <ros/time.h>
#include <Arduino.h>

ros::NodeHandle nh;

void commandCallback(const teensy::Command& cmd) {
    Vehicle.set_throttle_speed(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);
    if (cmd.estop) { Vehicle.estop_status = true; }
    if (cmd.profile) { Vehicle.run_profile(cmd.profile); }
}
ros::Subscriber<teensy::Command> sub("/Teensy/command", &commandCallback);

int main(int argc, char** argv){
    nh.initNode();

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.z = 0.0;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.y = 0.0;
    
    //ros::Publisher pub("odom", &odom);
    
    nh.subscribe(sub);
    //nh.advertise(pub);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time curr_time, last_time;
    curr_time = nh.now();
    last_time = curr_time;
    
    while (1) {

        nh.spinOnce();

        curr_time = nh.now();
        double v = Vehicle.get_speed_mps();
        double delta = Vehicle.get_steering_rad();
        double beta = atan( WHEELBASE_REAR_MM*tan(delta) / (WHEELBASE_FRONT_MM+WHEELBASE_REAR_MM) );
        double dt = curr_time.toSec() - last_time.toSec();
        
        x += v * cos(th+beta) * dt;
        y += v * sin(th+beta) * dt;
        double th_dot = v * delta / ((WHEELBASE_FRONT_MM+WHEELBASE_REAR_MM)/1000.0);
        th +=  th_dot * dt;

        odom_trans.header.stamp = curr_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = curr_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = th_dot;
        //pub.publish(&odom);

        last_time = curr_time;

    }

    return 0;
}
