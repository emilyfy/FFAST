cmake_minimum_required(VERSION 2.8.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME teensy)

include(cmake/pods.cmake)
include(cmake/teensy.cmake)

# BNO055 sources
set(BNO055_SRC src/BNO055/BNO055.cpp
               src/BNO055/i2c/i2c.cpp)

# main sources
set(MAIN_SRC src/main.cpp)

# build the teensy program
teensy_add_executable(${POD_NAME} ${MAIN_SRC} ${BNO055_SRC})
