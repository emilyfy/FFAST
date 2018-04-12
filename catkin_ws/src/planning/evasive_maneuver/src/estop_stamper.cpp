#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ilqr_msgs/BoolStamped.h>

ros::Publisher pub_estop;

void estopCb(const std_msgs::Bool::ConstPtr& msg)
{
    ilqr_msgs::BoolStamped estop;
    estop.header.stamp = ros::Time::now();
    estop.data = msg->data;
    pub_estop.publish(estop);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "estop_translate");
    ros::NodeHandle nh;
    
    pub_estop = nh.advertise<ilqr_msgs::BoolStamped>("/estop", 1);
    ros::Subscriber sub_estop = nh.subscribe("/commands/estop",1,estopCb);

    ros::spin();

    return 0;
}