#include <ros/ros.h>

#include "vesc_interface/multiplexer.h"
#include "vesc_interface/cmd_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  vesc_interface::Multiplexer mux(nh, pnh);
  vesc_interface::CmdPublisher command_publisher(nh, pnh);

  ros::spin();

  return 0;
}
