cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME teensy_bridge)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# collect sources
include(src/CMakeLists.txt)

# build LCM test programs
add_executable(${POD_NAME} ${${POD_NAME}_SOURCES})
pods_use_pkg_config_packages(${POD_NAME} lcm lcmtypes_${POD_NAME})

# make exeutables public
pods_install_executables(${POD_NAME})
