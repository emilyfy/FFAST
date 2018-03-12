#include <ros/ros.h>

#include "vesc_interface/odom_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  vesc_interface::OdomPublisher odom_publisher(nh, pnh);

  ros::spin();

  return 0;
}
