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
)

add_message_files(
  DIRECTORY msg
  FILES
  Orientation.msg
  Position.msg
  Odom.msg
)

add_service_files(
    FILES
    pid_gains.srv
    bool_call.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)




catkin_package(
    CATKIN_DEPENDS roscpp
    rospy std_msgs message_runtime
    sensor_msgs geometry_msgs nav_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    src/control/movement_controller.py
    src/perception/raw_odometry.py
    src/helpers/send_desired_movement.py
    src/control/keyboard_teleop.py
    src/perception/vision/lane_detector.py
    src/control/lane_follower.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
