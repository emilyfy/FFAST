cmake_minimum_required(VERSION 2.8.0)

set(POD_NAME teensy)

# pull in the pods macros. See cmake/pods.cmake for documentation
include(cmake/pods.cmake)
include(cmake/teensy.cmake)

# collect sources
include(src/CMakeLists.txt)

# build the teensy program
teensy_add_executable(${POD_NAME} ${TEENSY_SOURCES})
