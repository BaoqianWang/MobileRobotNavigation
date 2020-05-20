cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_robot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
  FILES
  smile_pwm.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
