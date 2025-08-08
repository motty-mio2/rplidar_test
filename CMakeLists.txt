

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
target_include_directories(${test_name} PRIVATE include)

find_package (gflags REQUIRED)
include_directories (${gflags_INCLUDE_DIR})

# rplidar SDK sources
file(GLOB RPLIDAR_SDK_SRC
    3rdparty/rplidar_sdk/sdk/src/*.cpp
    3rdparty/rplidar_sdk/sdk/src/arch/linux/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/dataunpacker/unpacker/*.cpp
    3rdparty/rplidar_sdk/sdk/src/hal/thread.cpp
)

add_executable(hello src/main.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(hello ${OpenCV_LIBS} pthread gflags)
