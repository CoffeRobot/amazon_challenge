Amazon Challenge
=====

Code for the competition in Seattle

# Contents

- [Installation](#markdown-header-installation)
- [Simulation](#markdown-header-simulation)
- [PR2](#markdown-header-pr2)
    - [Start and Stop](#markdown-header-start-and-stop)
    - [Object Tracking](#markdown-header-object-tracking)
    - [Moving the Robot](#markdown-header-moving-the-robot)

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

### Starting the Laser Tilt Controller

Example service call:
```
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 9 , amplitude: 1 , offset: 0 }}'
```

See [here](http://wiki.ros.org/pr2_mechanism_controllers/LaserScannerTrajController) for more details.

### Object Tracking 

For example on Aragorn:
```
export ROS_MASTER_URI=http://pr2-c1:11311
```

OBSOLETE: The old-style tracking cannot work with multiple cameras and is launched like this, depending on the camera:
```
roslaunch vision multi-rigid_pr2_head_mount_kinect.launch
roslaunch vision multi-rigid_pr2_l_forearm.launch
roslaunch vision multi-rigid_pr2_r_forearm.launch
```

You should now use the camera switching node instead:
```
roslaunch vision pr2_cam_switch.launch
```
This node will initiate to the *cheezit_big_original* object but we'll remove this in the future. Instead the active objects need to be set with the following service call:
```
rosservice call /simtrack/tracker_switch_objects [crayola_64_ct,mead_index_cards]
```
It accepts a list of objects that should be kept minimal to conserve GPU memory. So this list should be updated before moving to the next bin. The tracker will crash if you misspell the object name.

The camera can be selected with the following service call:
```
rosservice call /simtrack/tracker_switch_camera 0
```
The camera index can range from 0 to 2, as defined in [pr2_camera_topics.yaml](vision/launch/pr2_camera_topics.yaml):

- 0: kinect
- 1: left arm camera
- 2: right arm camera

Each frame, the list of reliably tracked objects (according to the tracker) is published as a string list on the topic:
```
/simtrack/reliably_tracked_objects
```

### Moving the Robot

To move the robot, look at [example.py](motion/example.py). It relies on [my_pr2.py](motion/my_pr2.py) which is available on the PR2 at:
```
~/amazon_challenge_ws/my_pr2.py
```

### Publish Aggregated Cloud

To create a cloud using the tilt scanner and the kinect these are the steps required:

- 0: Start the tilt scanner controller
```
rosservice call laser_tilt_controller/set_periodic_cmd '{ command: { header: { stamp: 0 }, profile: "linear" , period: 9 , amplitude: 1 , offset: 0 }}'
```
- 1: Launch the laser assembler: 
```
roslaunch vision laser_assembler.launch
```
- 2: Run the cloud aggregator: 
```
rosrun vision periodic_cloud_aggregator
```
The cloud is published to the topic periodic_cloud
