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

#include <sstream>
#include <fstream>

// global variables
// parameters
double steering_gain, steering_offset;
// values to be logged, updated in cb funcs
double steering_ang;
double long_vel;
double yaw_rate, yaw_rate_dot, lat_acc;
// cmd publisher and msg
ros::Publisher cmd_pub;
ackermann_msgs::AckermannDriveStamped cmd;

void servoCb(const std_msgs::Float64& msg)
{
    steering_ang = ( msg.data - steering_offset ) / steering_gain;
}

void odomCb(const nav_msgs::Odometry& msg)
{
    long_vel = msg.twist.twist.linear.x;
}

void imuCb(const sensor_msgs::Imu& msg)
{
    static double last_yaw_rate;
    static ros::Time last_time;
    ros::Time curr_time = ros::Time::now();
    
    yaw_rate = msg.angular_velocity.z;
    if (!last_yaw_rate) yaw_rate_dot = (yaw_rate - last_yaw_rate) / (curr_time - last_time).toSec();
    lat_acc = -1.0*msg.linear_acceleration.y;     //imu y-axis is -ve of car y-axis

    last_yaw_rate = yaw_rate;
    last_time = curr_time;
}

void pubTimerCb(const ros::TimerEvent& event)
{
    cmd_pub.publish(cmd);
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

    ros::Subscriber steer_sub = nh.subscribe("/sensors/servo_position_command",10,servoCb);
    ros::Subscriber odom_sub = nh.subscribe("/odom",10,odomCb);
    ros::Subscriber imu_sub = nh.subscribe("/imu",10,imuCb);
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/keyboard",5);

    std::ofstream outstream("/home/nvidia/Documents/ramp_steer.csv");
    outstream << "Mass (kg)," << mass << std::endl;
    outstream << "Wheelbase (m)," << wheelbase << std::endl;
    outstream << "Rotational inertia (kgm^2)," << I_zz << std::endl;
    outstream << "Rear axle to CoG (m)," << wheelbase_r << std::endl;
    outstream << "Lateral acc (m/s^2),Steering angle (rad),Yaw rate (rad/s),Longitudinal vel (m/s),Yaw rate dot (rad/s^2),m*L*a_y,I_zz*r_dot+m*L_r*a_y" << std::endl;

    int steering_ang_deg;

    sleep(5);          // wait for vesc driver to initialize
    cmd.drive.speed = 1.5;

    ros::Timer pub_timer = nh.createTimer(ros::Duration(0.1),pubTimerCb);
    usleep(400000);

    ros::Rate log_rate(20);
    
    for( steering_ang_deg = -30 ; steering_ang_deg <= 0 && ros::ok() ; steering_ang_deg++ ) {
        cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;
        cmd_pub.publish(cmd);
        ROS_INFO("Running at steering angle %d degrees.", steering_ang_deg);

        ros::Time start_time = ros::Time::now();
        while ( (ros::Time::now() - start_time).toSec() < 1.0 && ros::ok() ) {
            ros::spinOnce();
            outstream << lat_acc << "," << steering_ang << "," << yaw_rate << "," << long_vel << "," << yaw_rate_dot << "," ;
            outstream << mass*wheelbase*lat_acc << "," << I_zz*yaw_rate_dot+mass*wheelbase_r*lat_acc << std::endl;
            log_rate.sleep();
        }
    }
    
    // for( steering_ang_deg = 30 ; steering_ang_deg >= 0 && ros::ok() ; steering_ang_deg-- ) {
    //     cmd.drive.steering_angle = steering_ang_deg*M_PI/180.0;
    //     cmd_pub.publish(cmd);
    //     ROS_INFO("Running at steering angle %d degrees.", steering_ang_deg);

    //     ros::Time start_time = ros::Time::now();
    //     while ( (ros::Time::now() - start_time).toSec() < 1.0 ) {
    //         ros::spinOnce();
    //         outstream << lat_acc << "," << steering_ang << "," << yaw_rate << "," << long_vel << "," << yaw_rate_dot << "," ;
    //         outstream << mass*wheelbase*lat_acc << "," << I_zz*yaw_rate_dot+mass*wheelbase_r*lat_acc << std::endl;
    //         r.sleep();
    //     }
    // }

    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;
    cmd_pub.publish(cmd);
    sleep(5);

    outstream.close();
    return 0;
}
