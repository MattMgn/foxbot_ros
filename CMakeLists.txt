cmake_minimum_required(VERSION 2.8.3)
project(foxbot_ros)

## Find catkin macros and libraries
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf sensor_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_calib src/odom_calib.cpp)
target_link_libraries(odom_calib ${catkin_LIBRARIES})
