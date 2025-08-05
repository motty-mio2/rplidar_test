cmake_minimum_required(VERSION 3.10)
project(rplidar_test)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(hello src/main.cpp)
target_link_libraries(hello ${OpenCV_LIBS})
