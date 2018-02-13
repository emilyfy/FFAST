cmake_minimum_required(VERSION 2.8.3)

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")
add_library(ros_lib ${ros_src})

include_directories(${ROS_LIB_DIR})

# Use this command to add arduino libraries
#import_arduino_library();

add_teensy_executable(target1 main.cpp vehicle.cpp)
target_link_libraries(target1 ros_lib)
