// Ramp steer test to determine cornering stiffness
// Based on Sierra, Tseng, Jain & Peng (2006)
// Plot I_zz*r_dot+m*L_r*a_y against m*L*a_y
// find least squares error line
// gradient gives X1, y-intercept gives L(delta-L*r/u)*X2
// C_alpha_r = X2/X1 ; C_alpha_f = X1/(1-X1)*C_alpha_r

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/transform_datatypes.h>

#include <sstream>
#include <fstream>

// global variables
// parameters
double steering_gain, steering_offset;
// values to be logged, updated in cb funcs
double steering_ang;
double long_vel, lat_vel, yaw_rate;
double yaw_rate_imu, yaw_rate_dot, lat_acc;
double wheel_vel;
ackermann_msgs::AckermannDriveStamped cmd;

void servoCb(const std_msgs::Float64& msg)
{
    steering_ang = ( msg.data - steering_offset ) / steering_gain;
}

void poseCb(const nav_msgs::Odometry& msg)
{
    long_vel = msg.twist.twist.linear.x;
    lat_vel = msg.twist.twist.linear.y;
    yaw_rate = msg.twist.twist.angular.z;
}

void odomCb(const nav_msgs::Odometry& msg)
{
    wheel_vel = msg.twist.twist.linear.x/0.03235;
}

void imuCb(const sensor_msgs::Imu& msg)
{
    ros::Time curr_time = ros::Time::now();
    static ros::Time last_time = curr_time;
    
    yaw_rate_imu = msg.angular_velocity.z;
    static double last_yaw_rate = yaw_rate_imu;
    
    yaw_rate_dot = (yaw_rate - last_yaw_rate) / (curr_time - last_time).toSec();
    // lat_acc = -1.0*msg.linear_acceleration.y;     //imu y-axis is -ve of car y-axis
    lat_acc = msg.linear_acceleration.y;

    last_yaw_rate = yaw_rate;
    last_time = curr_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ramp_steer");
    ros::NodeHandle nh;

    // get required vehicle parameters
    double mass, I_zz, wheelbase, wheelbase_r;
    if (!nh.getParam("vehicle/mass", mass)) {
        ROS_FATAL("Parameter mass is required.");
        return 1;
    }
    if (!nh.getParam("vehicle/I_zz", I_zz)) {
        ROS_FATAL("Parameter I_zz is required.");
        return 1;
    }
    if (!nh.getParam("vehicle/wheelbase", wheelbase)) {
        ROS_FATAL("Parameter wheelbase is required.");
        return 1;
    }
    if (!nh.getParam("vehicle/wheelbase_r", wheelbase_r)) {
        ROS_FATAL("Parameter wheelbase_r is required.");
        return 1;
    }
    if (!nh.getParam("vesc_interface/steering_gain",steering_gain)) {
        ROS_FATAL("Parameter steering_gain is required.");
        return 1;
    }
    if (!nh.getParam("vesc_interface/steering_offset", steering_offset)) {
        ROS_FATAL("Parameter steering_offset is required.");
        return 1;
    }

    ros::Subscriber steer_sub = nh.subscribe("/sensors/servo_position_command",1,servoCb);
    ros::Subscriber pose_sub = nh.subscribe("/ekf_localization/odom",1,poseCb);
    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imuCb);
    ros::Subscriber odom_sub = nh.subscribe("/odom_fused",1,odomCb);
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",1);

    std::ofstream outstream("/home/emily/Documents/ramp_steer.csv");
    outstream << "Mass (kg)," << mass << std::endl;
    outstream << "Wheelbase (m)," << wheelbase << std::endl;
    outstream << "Rotational inertia (kgm^2)," << I_zz << std::endl;
    outstream << "Rear axle to CoG (m)," << wheelbase_r << std::endl;
    outstream << "Wheel Vel (rad/s),Steering angle (rad),Longitudinal vel (m/s),Lateral vel (m/s),Yaw rate (rad/s),Yaw rate by IMU (rad/s^2),Yaw rate dot (rad/s^2),Lateral acc (m/s^2),m*L*a_y,I_zz*r_dot+m*L_r*a_y" << std::endl;

    int steering_ang_deg;

    ros::Duration(5).sleep();          // wait for vesc driver to initialize
    ros::Rate log_rate(20);
    cmd.drive.speed = 3;
    
    for( steering_ang_deg = -30 ; steering_ang_deg <= 0 && ros::ok() ; steering_ang_deg++ ) {
        cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;
        cmd_pub.publish(cmd);
        ROS_INFO("Running at steering angle %d degrees.", steering_ang_deg);

        ros::Time start_time = ros::Time::now();
        while ( (ros::Time::now() - start_time).toSec() < 1.0 && ros::ok() ) {
            ros::spinOnce();
            outstream << wheel_vel << "," << steering_ang << "," << long_vel << "," << lat_vel << "," << yaw_rate << "," << yaw_rate_imu << "," << yaw_rate_dot << "," << lat_acc << "," ;
            outstream << mass*wheelbase*lat_acc << "," << I_zz*yaw_rate_dot+mass*wheelbase_r*lat_acc << std::endl;
            cmd_pub.publish(cmd);
            log_rate.sleep();
        }
    }

    // for( steering_ang_deg = 30 ; steering_ang_deg >= 0 && ros::ok() ; steering_ang_deg-- ) {
    //     cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;
    //     cmd_pub.publish(cmd);
    //     ROS_INFO("Running at steering angle %d degrees.", steering_ang_deg);

    //     ros::Time start_time = ros::Time::now();
    //     while ( (ros::Time::now() - start_time).toSec() < 1.0 && ros::ok() ) {
    //         ros::spinOnce();
    //         outstream << lat_acc << "," << steering_ang << "," << yaw_rate << "," << long_vel << "," << yaw_rate_dot << "," ;
    //         outstream << mass*wheelbase*lat_acc << "," << I_zz*yaw_rate_dot+mass*wheelbase_r*lat_acc << std::endl;
    //         cmd_pub.publish(cmd);
    //         log_rate.sleep();
    //     }
    // }

    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    outstream.close();
    return 0;
}
