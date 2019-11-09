# Universal Robot

This repository depends on [olinrobotics/hiro](https://github.com/olinrobotics/hiro). Please also clone that repository before compiling this package.

[![Build Status](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](http://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.


__Installation__

There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.


First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/universal_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile:

```bash
rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_make
catkin_make
```

___Using apt (Ubuntu, Debian)___

On supported Linux distributions (Ubuntu, up to 16.04 (Xenial), `i386` and `amd64`) and ROS versions:

```
sudo apt-get install ros-$ROS_DISTRO-universal-robot
```

replace `$ROS_DISTRO` with `hydro`, `indigo` or `kinetic`, depending on which ROS version you have installed.


___Building from Source___

There are releases available for ROS Hydro, Indigo and Kinetic. However, for the latest features and developments you might want to build the packages from source.

**NOTE**: please prefer using the binary release (see previous section) over building from source where possible. Source installs will not be automatically updated by new package releases and require more work to setup.

The following instructions assume that a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created at `$HOME/catkin_ws` and that the source space is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

In all other cases the packages will have to be build from sources in a Catkin workspace:

```
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```


__Usage__

___With real Hardware___
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```bash
roslaunch ur_bringup xamyab_bringup.launch [left_robot_ip:=IP_OF_THE_ROBOT] [left_reverse_port:=REVERSE_PORT] [right_robot_ip:=IP_OF_THE_ROBOT] [right_reverse_port:=REVERSE_PORT] [limited:=true]
```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


___MoveIt! with real Hardware___  
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


___Usage with Gazebo Simulation___  
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

___MoveIt! with a simulated robot___  
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
