// Sudden full brake test from a certain velocity to determine friction cofficient
// friction force = coeff * mg = braking force = ma
// a can be obtained from imu
// from time taken to stop, a = v/t
// from distance (most accurate?) a = v^2/2s
// friction coeff = a/g = v/(t*g) = v^2/(2*s*g)

#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <fstream>

#define RUN_SPEED_TIME 0.5
#define RUN_BRAKE_TIME 5

// global variables - values to be logged, updated in cb funcs
double vel;
double acc;
double pos;

void odomCb(const nav_msgs::Odometry& msg)
{
    vel = msg.twist.twist.linear.x;
}

void imuCb(const sensor_msgs::Imu& msg)
{
    acc = -1.0*msg.linear_acceleration.y;     //imu x-axis is -ve of car x-axis
}

void poseCb(const geometry_msgs::PoseStamped& msg)
{
    pos = msg.pose.position.x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sudden_brake");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string pose_topic;
    if (!pnh.getParam("pose_topic", pose_topic)) {
        pose_topic = "laser_scan_matcher/pose";
    }

    ros::Subscriber odom_sub = nh.subscribe("/odom",10,odomCb);
    ros::Subscriber imu_sub = nh.subscribe("/imu",10,imuCb);
    ros::Subscriber sub_pose = nh.subscribe(pose_topic,10,poseCb);

    ackermann_msgs::AckermannDriveStamped cmd;
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",5);
    ros::Publisher brake_pub = nh.advertise<std_msgs::Float64>("/commands/motor/brake",1);

    ros::Rate log_rate(20);

    std::ofstream outstream("/home/nvidia/Documents/friction_coeff.csv");

    sleep(5);          // wait for vesc driver to initialize

    // variables
    char str[10];
    float cmd_val;
    char cmd_c;
    std_msgs::Float64 brake_msg;
    brake_msg.data = 10.0;
    ros::Time start_time, curr_time;

    while (ros::ok())
    {
        printf("Enter speed (m/s) to run test in\n");
        scanf("%s", str);
        sscanf(str, "%f", &cmd_val);
        
        cmd.drive.speed = cmd_val;
        cmd_pub.publish(cmd);
        printf("Velocity set to %f m/s\n", cmd_val);

        outstream << "Test at " << cmd_val << "m/s" << std::endl;
        outstream << "Time (s),Time (ns),Position (m),Velocity (m/s),Acceleration (m/s^2)" << std::endl;

        start_time = ros::Time::now();
        curr_time = start_time;
        // run at set speed for the specified time
        while ( (curr_time - start_time).toSec() < RUN_SPEED_TIME && ros::ok() ) {
            curr_time = ros::Time::now();
            ros::spinOnce();
            outstream << curr_time.toSec() << "," << curr_time.toNSec() << "," << pos << "," << vel << "," << acc << std::endl;
            cmd_pub.publish(cmd);
            log_rate.sleep();
        }

        // sudden brake
        brake_pub.publish(brake_msg);
        start_time = ros::Time::now();
        curr_time = start_time;
        outstream << "Brake started at time " << curr_time.toSec() << "(s) or" << curr_time.toNSec() << "(ns)" << std::endl;
        while ( (curr_time - start_time).toSec() < RUN_BRAKE_TIME && ros::ok() ) {
            curr_time = ros::Time::now();
            ros::spinOnce();
            outstream << curr_time.toSec() << "," << curr_time.toNSec() << "," << pos << "," << vel << "," << acc << std::endl;
            brake_pub.publish(brake_msg);
            log_rate.sleep();
        }

        // completed
        printf("Test with speed %f finished.\n", cmd_val);
        outstream << "Test at " << cmd_val << "m/s" << "finished" << std::endl << std::endl;
        printf("Run another test? (y or n) \n");
        scanf("%s", str);
        sscanf(str, "%c", &cmd_c);
        if (cmd_c=='n' || cmd_c=='N') break;
    }

    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    sleep(5);

    outstream.close();
    return 0;
}
