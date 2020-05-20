# smile-mobile
Welcome! Smile-mobile is an small autonomous ground vehicle seeking to develop its capabilities to be an intelligent autonomous system capable of navigating various environments, from nice paved rounds to bumpy, rocky terrian. The Smile-mobile software platform is built on the robotic operating system (ROS).

### Project Structure
The ROS environment for smile-mobile is broken up into two seperate workspaces (catkin workspaces). The first workspace is the robot workspace which is contained in the directory **smile_mobile_robot_ws**. The robot workspace contains all of the software that runs on the robot (both physical and simulated); the software in this package should be agnostic of whether the robot is the physical or simulated robot. 

The second workspace is the simulation workspace which is contained in the **smile_mobile_sim_ws** directory. The simulation workspace contains all the contents required for building the simulated world in gazebo and glueing together the robots control software with the simulated robot model.

Each of the workspaces contain ROS packages for various functions.

General File Structure
```
>smile_mobile_robot_ws
  >src
    >smile_mobile_robot (package)
      >CMakeLists.txt
      >package.xml
      >src
      .
      .
      .
      
>smile_mobile_sim_ws
  >src
    >smile_mobile_sim (package)
      >CMakeLists.txt
      >package.xml
      >src
      .
      .
      .
```

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
### Launching the Gazebo Simulation
To launch the Gazebo environment, make sure to be in the **smile_mobile_sim_ws**. The simulation will launch the Husky robot. For more on the Husky robot, see [here](https://github.com/husky).

Launch robot in empty world
```cmd
roslaunch smile_mobile_sim smile_empty_world.launch
```

Launch robot in smile world
```cmd
roslaunch smile_mobile_sim smile_world.launch
```
