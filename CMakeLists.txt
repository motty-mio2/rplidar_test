cmake_minimum_required(VERSION 3.10)
project(rplidar_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(include)
# rplidar SDK include
include_directories(3rdparty/rplidar_sdk/sdk/include)
include_directories(3rdparty/rplidar_sdk/sdk/src)
include_directories(3rdparty/rplidar_sdk/sdk/src/hal)
include_directories(3rdparty/rplidar_sdk/sdk/src/arch/linux)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# rplidar SDK sources
file(GLOB RPLIDAR_SDK_SRC
    3rdparty/rplidar_sdk/sdk/src/*.cpp
    3rdparty/rplidar_sdk/sdk/src/arch/linux/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/unpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/hal/thread.cpp
)

add_executable(hello src/main.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(hello ${OpenCV_LIBS} pthread)
