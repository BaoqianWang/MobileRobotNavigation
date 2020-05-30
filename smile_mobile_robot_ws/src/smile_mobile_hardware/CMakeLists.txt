cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_hardware)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)


catkin_package(
    CATKIN_DEPENDS
    rospy std_msgs
    sensor_msgs geometry_msgs nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    src/hardware_interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

