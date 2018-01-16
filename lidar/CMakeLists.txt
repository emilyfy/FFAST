cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME lidar)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

include_directories(include lcmtypes/cpp/)

# Search for the hokuyoaist package
find_package(HokuyoAIST)

include_directories(${HokuyoAIST_INCLUDE_DIRECTORIES})
add_definitions(${HokuyoAIST_DEFINITIONS})


# build LCM test programs
add_executable(${POD_NAME} src/hokuyo_publisher.cpp)
target_link_libraries(${POD_NAME} ${HokuyoAIST_LIBRARIES})
pods_use_pkg_config_packages(${POD_NAME} lcm lcmtypes_${POD_NAME})

add_executable(${POD_NAME}_listener src/hokuyo_listener.cpp)
target_link_libraries(${POD_NAME}_listener ${HokuyoAIST_LIBRARIES})
pods_use_pkg_config_packages(${POD_NAME}_listener lcm lcmtypes_${POD_NAME})

# make executables public
pods_install_executables(${POD_NAME} ${POD_NAME}_listener)
