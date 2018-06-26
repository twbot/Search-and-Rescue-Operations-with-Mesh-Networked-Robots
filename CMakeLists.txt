cmake_minimum_required(VERSION 2.8.3)
project(ros_rover_move)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs genmsg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
#catkin_package()

#include_directories(include ${catkin_INCLUDE_DIRS})

##add_executable(ros_rover_move src/main.py)
##target_link_libraries(ros_rover_move ${catkin_LIBRARIES})
## add_dependencies(ros_rover_move ros_rover_move_generate_messages_cpp)

catkin_python_setup()