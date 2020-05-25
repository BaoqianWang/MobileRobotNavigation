cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    pid
)

add_message_files(
  DIRECTORY msg
  FILES
  Orientation.msg
  Position.msg
  Odom.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)




catkin_package(
    CATKIN_DEPENDS roscpp
    rospy std_msgs message_runtime pid
    sensor_msgs geometry_msgs nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    src/control/movement_controller.py
    src/perception/odometry.py
    src/helpers/send_desired_odom.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
