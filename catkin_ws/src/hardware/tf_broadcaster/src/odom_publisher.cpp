#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <teensy_msgs/Feedback.h>
#include <teensy_msgs/Command.h>

#define WHEELBASE_FRONT_MM   67.09
#define WHEELBASE_REAR_MM   189.91

double s;
double v;
double delta;

void feedbackCallback(const teensy_msgs::Feedback& msg)
{
    s = msg.odom_m;
    v = msg.vel_mps;
}

void commandCallback(const teensy_msgs::Command& msg)
{
    delta = msg.steering_rad;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber fdb_sub = nh.subscribe("/teensy/feedback",50,feedbackCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    nav_msgs::Odometry odom;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.y = 0.0;

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    bool odom_tf;
    if (!pnh.getParam("odom_tf", odom_tf)) {
        odom_tf = false;
    }
    else {
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.z = 0.0;

        double odom_covariance;
        if (pnh.getParam("odom_covariance", odom_covariance)) {
            for(int i=0;i<6;i++) {
                odom.pose.covariance[6*i+i] = odom_covariance;
                odom.twist.covariance[6*i+i] = odom_covariance;
            }
        }
    }

    // Start at origin
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double curr_odom;
    ros::Time curr_time, last_time;
    curr_time = ros::Time::now();
    last_time = curr_time;

    ros::Rate r(600);

    while (nh.ok()) {

        ros::spinOnce();

        curr_odom = s;
        static double last_odom = curr_odom;
        double ds = curr_odom - last_odom;
        curr_time = ros::Time::now();
        double dt = (curr_time - last_time).toSec();

        double beta = atan( WHEELBASE_REAR_MM*tan(delta) / (WHEELBASE_FRONT_MM+WHEELBASE_REAR_MM) );
        x += ds * cos(th+beta); // v * cos(th+beta) * dt;
        y += ds * sin(th+beta); // v * sin(th+beta) * dt;
        double th_dot = v * delta / ((WHEELBASE_FRONT_MM+WHEELBASE_REAR_MM)/1000.0);
        th +=  th_dot * dt;

        odom.header.stamp = curr_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = th_dot;
        odom_pub.publish(odom);

        if (odom_tf) {
            odom_trans.header.stamp = curr_time;
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster.sendTransform(odom_trans);
        }

        last_time = curr_time;
        last_odom = curr_odom;

        r.sleep();

    }

    return 0;
}
