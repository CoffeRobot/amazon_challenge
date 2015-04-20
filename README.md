Amazon Challenge
=====

Code for the competition in Seattle

# Contents

- [Installation](#installation)
- [Simulation](#simulation)
- [PR2](#pr2)
    - [Start and Stop](##start-and-stop)
    - [Object Tracking](##object-tracking)
    - [Moving the Robot](##moving-the-robot)

## Installation

Install wstool:
```
sudo apt-get install python-wstool
```

Create your workspace:
```
mkdir -p ~/catkin_ws/src
```

Copy the contents of [amazon_challenge.rosinstall](amazon_challenge.rosinstall) into a file ~/catkin_ws/src/.rosinstall
*Note: Bitbucket requires this README to be rendered at a specific commit for the file links to work (e.g. go to source and select the devel branch).*

Fetch the code:
```
cd ~/catkin_ws/src
wstool update
```

Install the dependencies:
```
cd ~/catkin_ws
sudo rosdep init # only if never run before
rosdep install --from-paths src --ignore-src
```

Build:
```
cd ~/catkin_ws
catkin_make
```

## Simulation

```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_moveit_config move_group.launch
```

## PR2

### Start and Stop

Start:
```
ssh user@pr2-c1
robot claim
robot start
```

Activate the Kinect with Yasemin's calibration:
```
cd ~/amazon_challenge_ws/
roslaunch kinect_yasemin/kinect_node.launch
```

Start moveit:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch pr2_ft_moveit_config move_group.launch
```

Stop:
```
robot stop
robot release
```

### Object Tracking 

For example on Aragorn:
```
export ROS_MASTER_URI=http://pr2-c1:11311
roslaunch vision multi-rigid_pr2_head_mount_kinect.launch
roslaunch vision multi-rigid_pr2_l_forearm.launch
roslaunch vision multi-rigid_pr2_r_forearm.launch
```

### Moving the Robot

To move the robot, look at [example.py](motion/example.py). It relies on [my_pr2.py](motion/my_pr2.py) which is available on the PR2 at:
```
~/amazon_challenge_ws/my_pr2.py
```
