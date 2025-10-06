cmake_minimum_required(VERSION 3.10)
project(rplidar_test)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(include)

# GoolgeTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

#テスト用ターゲット
file(GLOB TEST_SOURCES test/*.cpp)
foreach(test_src ${TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_include_directories(${test_name} PRIVATE include)
  target_link_libraries(${test_name} ${OpenCV_LIBS} gtest gtest_main)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

target_include_directories(${test_name} PRIVATE include)

# rplidar SDK include
include_directories(3rdparty/rplidar_sdk/sdk/include)
include_directories(3rdparty/rplidar_sdk/sdk/src)
include_directories(3rdparty/rplidar_sdk/sdk/src/hal)
include_directories(3rdparty/rplidar_sdk/sdk/src/arch/linux)

# Third party libraries
find_package (gflags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

include_directories (${gflags_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# rplidar SDK sources
file(GLOB RPLIDAR_SDK_SRC
    3rdparty/rplidar_sdk/sdk/src/*.cpp
    3rdparty/rplidar_sdk/sdk/src/arch/linux/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/unpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/hal/thread.cpp
)

file(GLOB LIDAR_TEST_SRC
  src/main.cpp
  src/lidar_obj.cpp
  src/zenoh_wrapper.cpp
)

add_executable(hello ${LIDAR_TEST_SRC} ${RPLIDAR_SDK_SRC})
target_link_libraries(hello ${OpenCV_LIBS} pthread gflags zenohcxx::zenohc )

