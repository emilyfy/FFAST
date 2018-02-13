#include <stdio.h>
#include <ros/ros.h>
#include <teensy_msgs/Command.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>

double goal, x;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    x = msg.pose.pose.position.x;
}

void goalCallback(const std_msgs::Float64& msg)
{
    goal = msg.data;
    ROS_INFO("Goal set to %f m.", goal);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "goal");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<teensy_msgs::Command>("/teensy/command",10);
    ros::Subscriber sub_pose = nh.subscribe("/laser_scan_matcher/pose",10,poseCallback);
    ros::Subscriber sub_goal = nh.subscribe("/goal",1,goalCallback);

    teensy_msgs::Command cmd;
    
    ros::Rate r(200);
    
    while (ros::ok())
    {
        if ( fabs(goal-x) < 0.03) {
            cmd.vel_mps = 0.0;
        }
        else if ( x > goal ) {
            cmd.vel_mps = 0.0;
        }
        else if ( goal > x) {
            cmd.vel_mps = 1.5;
        }

        pub_cmd.publish(cmd);
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

