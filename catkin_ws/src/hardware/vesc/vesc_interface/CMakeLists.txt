cmake_minimum_required(VERSION 2.8.3)
project(vesc_interface)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  ackermann_msgs
  vesc_msgs
)

catkin_package(
 INCLUDE_DIRS include
  CATKIN_DEPENDS
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  ackermann_msgs
  vesc_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(vesc_interface_node src/vesc_interface_node.cpp
                                   src/multiplexer.cpp
                                   src/cmd_publisher.cpp)
add_dependencies(vesc_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vesc_interface_node
  ${catkin_LIBRARIES}
)

add_executable(odom_publisher_node src/odom_publisher_node.cpp
                                   src/odom_publisher.cpp)
add_dependencies(odom_publisher_node ${catkin_EXPORTED_TARGETS})                                   
target_link_libraries(odom_publisher_node
  ${catkin_LIBRARIES}
)  

#############
## Install ##
#############

install(TARGETS odom_publisher_node vesc_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})