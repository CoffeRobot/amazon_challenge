Amazon Challenge
=====

Code for the competition in Seattle


Installation
------------

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