cmake_minimum_required(VERSION 2.8.3)
project(smile_mobile_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
)

catkin_package(

  CATKIN_DEPENDS rqt_gui rqt_gui_py

)


catkin_python_setup()

#Provide ability to rosrun the gui
install(PROGRAMS scripts/PID_Tuner scripts/Position_Plotter
                 scripts/Odometry_Display
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
        resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
        plugin_pid_tuner.xml
        plugin_position_plotter.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
