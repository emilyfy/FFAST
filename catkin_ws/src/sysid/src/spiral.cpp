#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <teensy_msgs/Command.h>
#include <teensy_msgs/Feedback.h>

#include <sstream>
#include <fstream>

#define WHEELBASE_M 0.257

double yaw_rate_rps;
double vel_mps;

void imuCb(const sensor_msgs::Imu& msg)
{
    yaw_rate_rps = msg.angular_velocity.z;
}

void fdbCb(const teensy_msgs::Feedback& msg)
{
    vel_mps = msg.vel_mps;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_spiral");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu",10,imuCb);
    ros::Subscriber fdb_sub = nh.subscribe("/teensy/feedback",10,fdbCb);

    teensy_msgs::Command cmd;
    ros::Publisher cmd_pub = nh.advertise<teensy_msgs::Command>("teensy/command",10);

    std::ofstream outstream("/home/nvidia/Documents/spiral_log.csv");
    outstream << "Servo angle (deg),Velocity (m/s),Yaw rate (rad/s),Steering angle (rad)" << std::endl;

    int servo_ang_deg;

    sleep(5);

    cmd.vel_mps = 1.5;
    cmd_pub.publish(cmd);
    usleep(400000);
    
    for( servo_ang_deg = -30 ; servo_ang_deg <= 0 && ros::ok() ; servo_ang_deg++ ) {

        cmd.steering_rad = servo_ang_deg*M_PI/180.0;
        cmd_pub.publish(cmd);
        ROS_INFO("Running at servo angle %d degrees.", servo_ang_deg);

        sleep(1);

        ros::spinOnce();
        outstream << servo_ang_deg << "," << vel_mps << "," << yaw_rate_rps << "," << WHEELBASE_M*yaw_rate_rps/vel_mps << std::endl;
    }

    //for( servo_ang_deg = 30 ; servo_ang_deg >= 0 && ros::ok() ; servo_ang_deg-- ) {

    //    cmd.steering_rad = servo_ang_deg*M_PI/180.0;
    //    cmd_pub.publish(cmd);
    //    ROS_INFO("Running at servo angle %d degrees.", servo_ang_deg);

    //    sleep(1);

    //    ros::spinOnce();
    //    outstream << servo_ang_deg << "," << vel_mps << "," << yaw_rate_rps << "," << WHEELBASE_M*yaw_rate_rps/vel_mps << std::endl;

    //}

    cmd.vel_mps = 0.0;
    cmd.steering_rad = 0.0;
    cmd_pub.publish(cmd);
    sleep(5);

    outstream.close();
    return 0;
}
