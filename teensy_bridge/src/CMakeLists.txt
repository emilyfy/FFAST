list(APPEND ${POD_NAME}_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
)

include_directories("${CMAKE_CURRENT_LIST_DIR}")
include("${CMAKE_CURRENT_LIST_DIR}/SerialComms/CMakeLists.txt")
