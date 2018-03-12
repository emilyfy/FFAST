#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#define B_TOGGLE_ACTIVATION  buttons[0]
#define B_BRAKE              buttons[1]
#define B_STOP              buttons[5]
#define A_VEL                axes[4]
#define A_STEER              axes[0]

#define MAX_VEL_MPS     3.0
#define MIN_VEL_MPS    -2.0
#define MAX_STEER_RAD (M_PI/6)

ros::Publisher cmd_pub;
ros::Publisher brake_pub;
ros::Publisher estop_pub;

void joyCb(const sensor_msgs::Joy& msg)
{
    static bool active = false;
    static ackermann_msgs::AckermannDriveStamped cmd;
    
    if (msg.B_TOGGLE_ACTIVATION)
    {
        active = !active;
        if (active) { ROS_INFO("Joy activated."); }
        else { ROS_INFO("Joy deactivated."); }
    }

    static std_msgs::Bool estop_msg;
    if (msg.B_STOP)
    {
        estop_msg.data = true;
        estop_pub.publish(estop_msg);
    }
    else if (estop_msg.data)
    {
        estop_msg.data = false;
        estop_pub.publish(estop_msg);
    }

    if (active)
    {
        if (msg.B_BRAKE)
        {
            // sending brake command to vesc directly
            // is there a way to pass through mux
            std_msgs::Float64 brake;
            brake.data = 5.0;          // brake with 5A
            brake_pub.publish(brake);
        }

        else if (msg.A_VEL>0) {
            cmd.drive.speed = msg.A_VEL*msg.A_VEL*msg.A_VEL*MAX_VEL_MPS;
        } else if (msg.A_VEL<0) {
            cmd.drive.speed = msg.A_VEL*msg.A_VEL*msg.A_VEL*-1*MIN_VEL_MPS;
        } else {
            cmd.drive.speed = 0.0;
        }

        cmd.drive.steering_angle = msg.A_STEER*MAX_STEER_RAD;

        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_listener");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe("/joy",10,joyCb);
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/joy", 10);
    brake_pub = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);
    estop_pub = nh.advertise<std_msgs::Bool>("/commands/estop",1);
    
    ros::spin();
    return 0;

}
