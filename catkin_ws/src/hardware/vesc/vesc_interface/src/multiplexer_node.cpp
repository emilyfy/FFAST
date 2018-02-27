#include <ros/ros.h>

#include "vesc_interface/multiplexer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multiplexer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    vesc_interface::Multiplexer mux(nh, pnh);

    ros::spin();
    
    return 0;
}
