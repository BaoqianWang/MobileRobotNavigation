# smile-mobile
Welcome! Smile-mobile is an small autonomous ground vehicle seeking to develop its capabilities to be an intelligent autonomous system capable of navigating various environments, from nice paved rounds to bumpy, rocky terrian. The Smile-mobile software platform is built on the robotic operating system (ROS).

### Project Structure
The ROS environment for smile-mobile is broken up into two seperate workspaces (catkin workspaces). The first workspace is the robot workspace which is contained in the directory **smile_mobile_robot_ws**. The robot workspace contains all of the software that runs on the robot (both physical and simulated); the software in this package should be agnostic of whether the robot is the physical or simulated robot. 

The second workspace is the simulation workspace which is contained in the **smile_mobile_sim_ws** directory. The simulation workspace contains all the contents required for building the simulated world in gazebo and glueing together the robots control software with the simulated robot model.

Each of the workspaces contain ROS packages for various functions.

### Building the Project
First navigate into the **smile_mobile_robot_ws** directory. Run the following
```cmd
catkin_make
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source devel/setup.bash
```
Now navigate into the **smile_mobile_sim_ws** directory and run the following. **NOTE**: Notice the sourcing order 
```cmd
catkin_make
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source devel/setup.bash
```

Note: For some users, it maybe necessary to run **chmod +x [Executable].py** for any executable python scripts in the package.

### Launching the Gazebo Simulation
To launch the Gazebo environment, make sure to be in the **smile_mobile_sim_ws**.

Launch robot in empty world
```cmd
roslaunch smile_mobile_gazebo smile_mobile_world.launch gui:=true
roslaunch smile_mobile_control smile_control.launch
```

The robot can be controlled by sending PWM values (range [-255, 255]) to each motor in the order motor_1, motor_2, motor_3, motor_4.
```cmd
//Drive the robot straight forward example
rostopic pub /smile/pwm std_msgs/Int16MultiArray '{data: [100, 100, 100, 100]}'

//Turn the robot example
rostopic pub /smile/pwm std_msgs/Int16MultiArray '{data: [50, -50, -50, 50]}'
```
