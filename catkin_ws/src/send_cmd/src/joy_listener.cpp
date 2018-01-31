#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teensy/Command.h>

#define B_TOGGLE_ACTIVATION  buttons[0]
#define A_VEL                axes[4]
#define A_STEER              axes[0]

#define MAX_VEL_MPS    12.0
#define MIN_VEL_MPS    -6.0
#define MAX_STEER_RAD (M_PI*0.25)

ros::Publisher cmd_pub;

void joyCb(const sensor_msgs::Joy& msg)
{
    static bool active = false;
    static teensy::Command cmd;
    
    if (msg.B_TOGGLE_ACTIVATION)
    {
        active = !active;
        if (active) { ROS_INFO("Joy activated."); }
        else { ROS_INFO("Joy deactivated."); }
    }

    if (active)
    {
        if (msg.A_VEL>0) {
            cmd.vel_mps = msg.A_VEL*MAX_VEL_MPS;
        }
        else {
            cmd.vel_mps = msg.A_VEL*-1*MIN_VEL_MPS;
        }

        cmd.steering_rad = msg.A_STEER*MAX_STEER_RAD;

        cmd_pub.publish(cmd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_listener");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe("/joy",10,joyCb);
    cmd_pub = nh.advertise<teensy::Command>("teensy/command", 10);
    
    ros::spin();
    return 0;

}
