cmake_minimum_required(VERSION 3.16)
project(t265_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_path(LIBREALSENSE2_INCLUDE_DIR librealsense2/rs.hpp
          PATHS /usr/local/include /usr/include)
find_library(LIBREALSENSE2_LIB
             NAMES librealsense2.so librealsense2 realsense2
             PATHS /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib)
if(NOT LIBREALSENSE2_INCLUDE_DIR OR NOT LIBREALSENSE2_LIB)
    message(FATAL_ERROR
            "librealsense2 not found (looked for realsense2/rs.hpp and "
            "librealsense2.so). Install the T265-patched SDK first.")
endif()

set(SOURCES
    src/main.cpp
    src/source.cpp
    src/sdk_source.cpp
    src/gui_window.cpp
    src/trajectory_canvas.cpp
    src/imu_strip.cpp
)

add_executable(t265-demo ${SOURCES})
target_include_directories(t265-demo PRIVATE src ${LIBREALSENSE2_INCLUDE_DIR})
target_link_libraries(t265-demo PRIVATE Qt6::Widgets ${LIBREALSENSE2_LIB})

find_package(rclcpp QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
if(TARGET rclcpp::rclcpp AND TARGET nav_msgs::nav_msgs
   AND TARGET sensor_msgs::sensor_msgs)
    target_sources(t265-demo PRIVATE src/ros_source.cpp)
    target_compile_definitions(t265-demo PRIVATE DEMO_HAS_ROS=1)
    target_link_libraries(t265-demo PRIVATE rclcpp::rclcpp
                        nav_msgs::nav_msgs sensor_msgs::sensor_msgs)
    message(STATUS "t265-demo: ROS mode enabled (rclcpp found)")
else()
    message(STATUS "t265-demo: rclcpp not found, building SDK-only mode")
endif()
