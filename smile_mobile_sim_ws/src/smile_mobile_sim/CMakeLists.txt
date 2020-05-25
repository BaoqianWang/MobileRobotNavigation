cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_sim)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    gazebo_ros
    husky_gazebo
    smile_mobile_robot
)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS})


catkin_package(DEPENDS
    rospy
    std_msgs
    geometry_msgs
    gazebo_ros
    husky_gazebo
    smile_mobile_robot
)

install(DIRECTORY
    launch
    worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
    src/wheel_driver.py
    src/cvt_odom_data.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
