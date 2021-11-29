cmake_minimum_required(VERSION 3.0.2)
project(walker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})