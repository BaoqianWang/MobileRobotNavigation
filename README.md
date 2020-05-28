# smile-mobile
Welcome! Smile-mobile is an small autonomous ground vehicle seeking to develop its capabilities to be an intelligent autonomous system capable of navigating various environments, from nice paved rounds to bumpy, rocky terrian. The Smile-mobile software platform is built on the robotic operating system (ROS).

### Project Structure
The ROS environment for smile-mobile is broken up into two seperate workspaces (catkin workspaces). The first workspace is the robot workspace which is contained in the directory **smile_mobile_robot_ws**. The robot workspace contains all of the software that runs on the robot (both physical and simulated); the software in this package should be agnostic of whether the robot is the physical or simulated robot. 

The second workspace is the simulation workspace which is contained in the **smile_mobile_sim_ws** directory. The simulation workspace contains all the contents required for building the simulated world in gazebo and glueing together the robots control software with the simulated robot model.

Each of the workspaces contain ROS packages for various functions.

### Building the Project
First navigate into the **smile_mobile_robot_ws** directory. Run the following
```cmd
catkin_init_workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
Now navigate into the **smile_mobile_sim_ws** directory and run the following. **NOTE**: Notice the sourcing order 
```cmd
catkin_init_workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
Now, you should added the sourcing lines to the **.bashrc** so that the correct ROS_PACKAGE_PATH is set each time a new terminal is opened. Open the files with **vim ~/.bashrc**. At the end of the file, add

```cmd
source /opt/ros/melodic/setup.bash
source ~/smile-mobile/smile_mobile_robot_ws/devel/setup.bash
source ~/smile-mobile/smile_mobile_sim_ws/devel/setup.bash
```


Note: For some users, it maybe necessary to run **chmod +x [Executable].py** for any executable python scripts in the package.

### Launching the Gazebo Simulation

Launching the Smile Mobile Robot in Gazebo simulation can all be done by running the following launch file within the package **smile_mobile_robot**. This simulation launchs the necessary simulation artifacts for gazebo alongside the robots movement controllers and raw odometry estimation.

Launch robot in empty world (will in future launch a custom world.)
```cmd
roslaunch smile_mobile_robot main_sim.launch
```

The robot can be controlled by interfacing with the movement controller that executes velocity and steering controls. The following helper node allows to a desired velocity and steering angle. The first parameter is the velocity (current range ~[-0.5, 0.5]m/s). The second parameter is the steering angle (range [-pi, pi]).

```cmd
rosrun smile_mobile_robot send_desired_movement --movement 0.4 1.579
```

The position estimated by the raw odometry node can viewed with the **Position_Plotter** Qt plugin.

```cmd
rosrun smile_mobile_gui Position_Plotter
```

Note: If any issues are discovered, please set an issue inquiry in the github **issues** tab for this repo. Thanks!
