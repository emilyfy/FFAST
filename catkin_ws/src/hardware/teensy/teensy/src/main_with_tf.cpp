#include "vehicle.h"

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <teensy_msgs/Command.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

void commandCallback(const teensy_msgs::Command& cmd) {
    // Set throttle speed and steering angle according to command.
    Vehicle.set_velocity_setpoint(cmd.vel_mps);
    Vehicle.set_steering_angle(cmd.steering_rad);

    // Set estop flag if commanded by Jetson.
    if (cmd.estop) { estop_isr(); }
}

int main(int argc, char** argv){

    ros::NodeHandle nh;
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    ros::Subscriber<teensy_msgs::Command> sub("/teensy/command", &commandCallback);
    nh.subscribe(sub);

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
    
    ros::Publisher pub("/odom", &odom);
    nh.advertise(pub);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double curr_odom, last_odom;
    last_odom = Vehicle.get_odom_m();
    
    ros::Time curr_time, last_time;
    curr_time = nh.now();
    last_time = curr_time;
    
    while (!Vehicle.estop_status) {

        curr_time = nh.now();
        curr_odom = Vehicle.get_odom_m();
        double ds = curr_odom - last_odom;
        double v = Vehicle.get_speed_mps();
        double delta = Vehicle.get_steering_rad();
        double beta = atan( WHEELBASE_REAR_MM*tan(delta) / (WHEELBASE_FRONT_MM+WHEELBASE_REAR_MM) );
        double dt = curr_time.toSec() - last_time.toSec();
        
        x += ds * cos(th+beta); // v * cos(th+beta) * dt;
        y += ds * sin(th+beta); // v * sin(th+beta) * dt;
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
        pub.publish(&odom);

        nh.spinOnce();
        delay(5);
        Vehicle.update_pid();
        last_time = curr_time;
        last_odom = curr_odom;

    }

    odom.header.stamp = nh.now();
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.angular.z = 0;
    pub.publish(&odom);
    nh.logerror("Estop on Teensy.");

    return 0;
}
