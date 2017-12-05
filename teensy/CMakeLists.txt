cmake_minimum_required(VERSION 2.8.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME teensy)

include(cmake/pods.cmake)
include(cmake/teensy.cmake)

# BNO055 sources
set(BNO055_SRC src/BNO055/BNO055.cpp
               src/BNO055/i2c/i2c.cpp)

# PWMServo sources
set(PWM_SRC src/PWMServo/PWMServo.cpp)

# main sources
set(MAIN_SRC src/main.cpp src/vehicle.cpp)

# build the teensy program
teensy_add_executable(${POD_NAME} ${BNO055_SRC} ${PWM_SRC} ${MAIN_SRC})
