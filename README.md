# Universal Robot

This repository depends on [olinrobotics/hiro](https://github.com/olinrobotics/hiro). Please also clone that repository before compiling this package.

[![Build Status](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](http://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot


__Installation from Source__  
There are releases available for ROS Hydro and ROS Indigo. However, for the latest features and developments you might want to install from source.

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/universal_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile:

```bash
rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_make
catkin_make
```

---

__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```bash
roslaunch ur_bringup xamyab_bringup.launch [left_robot_ip:=IP_OF_THE_ROBOT] [left_reverse_port:=REVERSE_PORT] [right_robot_ip:=IP_OF_THE_ROBOT] [right_reverse_port:=REVERSE_PORT] [limited:=true]
```

A simple test script that moves the robot to predefined positions can be executed like this:

```bash
rosrun ur_driver test_move.py
```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


__MoveIt! with real Hardware__  
Additionally, you can use MoveIt! to control the robot.  
There exist MoveIt! configuration packages for both robots.  

For setting up the MoveIt! nodes to allow motion planning run:

```bash
roslaunch xamyab_moveit_config xamyab_moveit_planning_execution.launch grippers:=GRIPPERS_CODE [limited:=true]
```

GRIPPERS_CODE:
- 0 = No gripper
- 1 = Gripper for left arm only
- 2 = Gripper for right arm only
- 3 = Grippers for both arms

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```bash
roslaunch xamyab_moveit_config moveit_rviz.launch
```


**NOTE:**
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited'.


---

__Usage with Gazebo Simulation__  
There are launch files available to bringup a simulated robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the simulated robot in Gazebo, run:

```bash
roslaunch ur_gazebo xamyab.launch grippers:=GRIPPERS_CODE [limited:=true]
```

GRIPPERS_CODE:
- 0 = No gripper
- 1 = Gripper for left arm only
- 2 = Gripper for right arm only
- 3 = Grippers for both arms

__MoveIt! with a simulated robot__  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```bash
roslaunch xamyab_moveit_config xamyab_moveit_planning_execution.launch grippers:=GRIPPERS_CODE [limited:=true]
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```bash
roslaunch xamyab_moveit_config moveit_rviz.launch
```


**NOTE:** 
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited'. 

