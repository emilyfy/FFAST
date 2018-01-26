cmake_minimum_required(VERSION 2.8.3)
project(teensy)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    ros_teensy
    rosserial_client
    rosserial_arduino
    message_generation
)

add_message_files(
    FILES
    Command.msg
    Feedback.msg
)

generate_messages(
    DEPENDENCIES
)

catkin_package(
    CATKIN_DEPENDS message_runtime
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
