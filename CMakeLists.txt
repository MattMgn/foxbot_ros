cmake_minimum_required(VERSION 2.8.3)
project(foxbot_ros)

## Find catkin macros and libraries
## Find catkin and any catkin packages
## find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs tf sensor_msgs nav_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs tf sensor_msgs geometry_msgs Eigen3)

## Declare a catkin package
catkin_package()

# need to be uncomment if cmake cannot find eigen
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(odom_calib src/odom_calib.cpp)
add_executable(docking src/docking.cpp)

target_link_libraries(odom_calib ${catkin_LIBRARIES})
target_link_libraries(docking ${catkin_LIBRARIES})

