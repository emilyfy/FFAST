cmake_minimum_required(VERSION 2.8.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME teensy)

include(cmake/pods.cmake)
include(cmake/teensy.cmake)

# main sources
set(MAIN_SRC src/main.cpp src/vehicle.cpp)

# build the teensy program
teensy_add_executable(${POD_NAME} ${MAIN_SRC})
