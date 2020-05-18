cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_sim)

find_package(catkin REQUIRED COMPONENTS gazebo_ros husky_gazebo)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  DEPENDS gazebo_ros husky_gazebo
)

install(DIRECTORY launch worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})