cmake_minimum_required(VERSION 2.8.3)
project(ilqr_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  ackermann_msgs
  message_generation
)

add_message_files(
  FILES
  State.msg
  IlqrInput.msg
  IlqrOutput.msg
  BoolStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ackermann_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 std_msgs
 geometry_msgs
 ackermann_msgs
 message_runtime
)