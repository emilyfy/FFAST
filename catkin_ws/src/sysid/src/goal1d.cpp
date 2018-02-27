#include <stdio.h>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

double goal, x;

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
    x = msg.pose.position.x;
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
    ros::NodeHandle pnh("~");

    std::string pose_topic;
    if (!pnh.getParam("pose_topic", pose_topic)) {
        pose_topic = "laser_scan_matcher/pose";
    }

    ros::Publisher pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/commands/keyboard",5);
    ros::Subscriber sub_pose = nh.subscribe(pose_topic,10,poseCallback);
    ros::Subscriber sub_goal = nh.subscribe("/goal",1,goalCallback);

    ackermann_msgs::AckermannDriveStamped cmd;
    
    ros::Rate r(200);
    
    while (ros::ok())
    {
        if ( fabs(goal-x) < 0.03) {
            cmd.drive.speed = 0.0;
        }
        else if ( x > goal ) {
            cmd.drive.speed = 0.0;
        }
        else if ( goal > x) {
            cmd.drive.speed = 1.5;
        }

        pub_cmd.publish(cmd);
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

