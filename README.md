# HomeServiceRobot

This is the capstone ROS project from the Udacity [Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209) course.

## Overview

The robot's task is to pickup a package and deliver it to a goal location. The robot utilizes a map for localization and navigation. The map was initially created using the gmapping ROS package based on Simultaneous Localization and Mapping algorithm ([SLAM](http://wiki.ros.org/gmapping)). The robot utilizes the Adaptive Monte Carlo Localization ROS package ([AMCL](http://wiki.ros.org/amcl)) to track the pose of the robot against a known map.

### License

The source code is released under an [MIT license.](https://opensource.org/licenses/MIT)

**Author: Andres Campos**

The packages: ```add_markers```, ```pick_objects```, ```slam_gmapping```, ```amcl```,```move_base```, ```navigation``` and ```turtlebot_gazebo``` have been tested under  [ROS](https://www.ros.org/) Kinetic on Ubuntu 16.04. This code is for personal learning and any fitness for a particular purpose is disclaimed.

## Installation

### Installation from Packages
To install all packages from this repository as Debian packages use

```sudo apt-get install ros-kinetic-...```

Or use ```rosdep```:

```sudo rosdep install --from-paths src```

### Building from Source

#### Dependencies
* [Robotics Operating System (ROS)](https://www.ros.org/) (middleware for robotics)

```sudo rosdep install --from-paths src```

* [Adaptive Monte Carlo Localization (amcl)](http://wiki.ros.org/amcl) (probabilistic localization system for a robot moving in 2D)

```sudo apt-get install ros-kinetic-amcl```

* [move_base](http://wiki.ros.org/move_base) (allows the user to move the robot to a desired position and orientation)

```sudo apt-get install ros-kinetic-move-base```

* [navigation](http://wiki.ros.org/amcl) (2D navigation stack that outputs a velocity command to a mobile base given odometry information and a target pose)

```sudo apt-get install ros-kinetic-navigation```

* [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo) (Gazebo launch and world files for TurtleBot simulation)

```
cd /catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_simulator
cd /catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_gazebo
```
* [gmapping](http://wiki.ros.org/gmapping) (creates a 2D occupancy grid map from laser and pose data collected by a mobile robot)

```
cd /catkin_ws/src/slam_gmapping
git clone https://github.com/ros-perception/slam_gmapping.git
```

* [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop) (allows the user to teleoperate a robot)

```
cd /catkin_ws/src/slam_gmapping
git clone  https://github.com/turtlebot/turtlebot.git
```

#### Building
To build from source, clone the latest version from this repository into your catkin workspace and compile this package using
```
cd catkin_workspace/src
git clone https://github.com/acampos074/HomeServiceRobot.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

## Usage

Navigate to the ```/src/scripts``` directory, and launch the shell script:

```./home_service.sh```


## Launch files
* **world.launch**: Gazebo launch and world file
* **amcl_demo.launch**: probabilistic localization system
* **view_navigation.launch**: 3D visualizer

## Nodes

### **add_markers**

**Subscribed Topics**
* **```/pick_objects/marker_state```**[(std_msgs/Bool)](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)

  Tracks the boolean state of the marker. Set to false if the package is not in its pickup location.
* **```/amcl_pose```**[(geometry_msgs/PoseWithCovarianceStamped)](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html)

  Reads the estimated X & Y coordinates of the mobile robot from the AMCL algorithm.

**Published Topics**
* **```/visualization_marker```** [(visualization_msgs/Marker)](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html)

    Displays the shape, size, color and position of a marker.

### **pick_objects**

**Published Topics**
* **```/pick_objects/marker_state```** [(std_msgs/Bool)](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)

   Tracks the boolean state of the package. Set to false if the package is not in it's pickup location.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/acampos074/HomeServiceRobot/issues)
