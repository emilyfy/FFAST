#include "vmu931/vmu931.h"

#include <ros/ros.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "vmu931");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Vmu931 vmu931(nh, pnh);

    vmu931.spin();

}