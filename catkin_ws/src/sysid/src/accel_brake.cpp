// Sudden full brake test from a certain velocity to determine friction cofficient
// friction force = coeff * mg = braking force = ma
// a can be obtained from imu? differentiate pose?
// from time taken to stop, a = v/t
// from distance (most accurate?) a = v^2/2s
// friction coeff = a/g = v/(t*g) = v^2/(2*s*g)

// Accelerating and braking to determine longitudinal stiffness
// Based on Miller Youngberg Millie Schweizer Gerdes (2001)
// Plot a_x against omega/v_x
// Find least squares error line
// gradient gives C_x*r_e/m, y-intercept gives C_x/m
// r_e : effective wheel radius

#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <fstream>

#define RUN_SPEED_TIME 1
#define RUN_BRAKE_TIME 5

// global variables - values to be logged, updated in cb funcs
double pos, vel, acc;
double acc_imu;
double wheel_vel;

void poseCb(const nav_msgs::Odometry& msg)
{
    pos = msg.pose.pose.position.x;
    vel = msg.twist.twist.linear.x;

    static double prev_vel = vel;
    static ros::Time prev_time = msg.header.stamp;
    acc = (vel-prev_vel)/(msg.header.stamp-prev_time).toSec();
    prev_vel = vel;
    prev_time = msg.header.stamp;
}

void imuCb(const sensor_msgs::Imu& msg)
{
    // acc_imu = msg.linear_acceleration.x * -1.0;
    acc_imu = msg.linear_acceleration.x;    
}

void odomCb(const nav_msgs::Odometry& msg)
{
    wheel_vel = msg.twist.twist.linear.x/0.03235;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accel_brake");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber pose_sub = nh.subscribe("/ekf_localization/odom",10,poseCb);
    ros::Subscriber imu_sub = nh.subscribe("/imu",10,imuCb);
    ros::Subscriber odom_sub = nh.subscribe("/odom",10,odomCb);

    ackermann_msgs::AckermannDriveStamped cmd;
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",5);
    std_msgs::Float64 brake_msg;
    ros::Publisher brake_pub = nh.advertise<std_msgs::Float64>("/commands/motor/brake",1);

    ros::Rate log_rate(20);

    std::ofstream outstream("/home/nvidia/Documents/accel_brake.csv");

    // variables
    char str[10];
    char cmd_c;
    float cmd_val;
    brake_msg.data = 10.0;
    ros::Time start_time, curr_time;

    ros::Duration(5).sleep();          // wait for vesc driver to initialize

    while (ros::ok())
    {
        printf("Enter speed (m/s) to run test in\n");
        scanf("%s", str);
        sscanf(str, "%f", &cmd_val);
        
        cmd.drive.speed = cmd_val;
        cmd_pub.publish(cmd);
        printf("Velocity set to %f m/s\n", cmd_val);

        outstream << "Test at " << cmd_val << "m/s" << std::endl;
        outstream << "Time (s),Time (ns),Position (m),Velocity (m/s),Acceleration (m/s^2),Wheel velocity (rad/s),IMU accel (m/s^2)" << std::endl;

        start_time = ros::Time::now();
        curr_time = start_time;
        // run at set speed for the specified time
        while ( (curr_time - start_time).toSec() < RUN_SPEED_TIME && ros::ok() ) {
            curr_time = ros::Time::now();
            ros::spinOnce();
            outstream << curr_time.toSec() << "," << curr_time.toNSec() << "," << pos << "," << vel << "," << acc << "," << wheel_vel << "," << acc_imu << std::endl;
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
            outstream << curr_time.toSec() << "," << curr_time.toNSec() << "," << pos << "," << vel << "," << acc << "," << wheel_vel << "," << acc_imu << std::endl;
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

    outstream.close();
    ros::Duration(5).sleep();
    return 0;
}
