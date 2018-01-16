cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME vmu931)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# include header files for this pod
include_directories(include lcmtypes/c/)

set(VMU931_SOURCES src/main.c src/vmu_utils.c)
set(LISTENER_SOURCES src/listener.c)

# Enable -g debug flags
#set(CMAKE_BUILD_TYPE Debug)

# build LCM program
add_executable(${POD_NAME} ${VMU931_SOURCES})
add_executable(${POD_NAME}_listener ${LISTENER_SOURCES})
pods_use_pkg_config_packages(${POD_NAME} lcm lcmtypes_${POD_NAME})
pods_use_pkg_config_packages(${POD_NAME}_listener lcm lcmtypes_${POD_NAME})

# make executable public
pods_install_executables(${POD_NAME} ${POD_NAME}_listener)
