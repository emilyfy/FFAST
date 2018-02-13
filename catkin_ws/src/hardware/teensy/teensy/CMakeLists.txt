cmake_minimum_required(VERSION 2.8.3)
project(teensy)

find_package(catkin REQUIRED COMPONENTS
    ros_teensy
    rosserial_client
    rosserial_arduino
    teensy_msgs
)

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT  make_libraries.py
)

rosserial_configure_client(
    DIRECTORY src 
    TOOLCHAIN_FILE ${ROS_TEENSY_TOOLCHAIN}
)

rosserial_add_client_target(src target1_Firmware ALL)
