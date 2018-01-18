cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME serial_lcm_interface)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build LCM test programs
add_executable(serial src/main.c)
target_link_libraries(serial m)
pods_use_pkg_config_packages(serial lcm lcmtypes_${POD_NAME})
add_executable(serial_send src/send_message.c)
pods_use_pkg_config_packages(serial_send lcm lcmtypes_${POD_NAME})
add_executable(serial_listener src/listener.c)
pods_use_pkg_config_packages(serial_listener lcm lcmtypes_${POD_NAME})

# make exeutables public
pods_install_executables(serial serial_send serial_listener)
