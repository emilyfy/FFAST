cmake_minimum_required(VERSION 2.8.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME teensy)
include(cmake/pods.cmake)
include(cmake/teensy.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build two LCM test programs
teensy_add_executable(${POD_NAME} src/main.cpp)

#add_executable(listener src/listener.c)
#pods_use_pkg_config_packages(listener lcm lcmtypes_${POD_NAME})

# make executables public
#pods_install_executables(send_message listener)
