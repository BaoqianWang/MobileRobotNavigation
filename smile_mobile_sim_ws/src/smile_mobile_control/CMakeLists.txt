cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_control)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
    robot_state_publisher
)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(DEPENDS
  controller_manager
  joint_state_controller
  robot_state_publisher
)
