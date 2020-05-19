cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

