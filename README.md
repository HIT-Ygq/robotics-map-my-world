# Robotics Simulation - Where Am I 

In this robotics simulation in [gazebo](http://gazebosim.org/) a custom *mobile robot* applies the Adaptive Monte Carlo Localization algorithm (AMCL), to determine it's position and pose in the environment (i.e. world). The AMCL ROS package improves the localization process of the MCL algorithm, due to dynamically adjusting the number of particles over a period 
of time, as the robot navigates in the map. In addition, the robot applies laser scanning ([Lidar](https://en.wikipedia.org/wiki/Lidar) to get data for mapping the environment. The ROS version that gets used in the project is [ROS-Kinetic](https://www.ros.org/). The 3D visualization tool [Rviz tool for ROS](http://wiki.ros.org/rviz) gets used to ... (to be continued) 

### Prerequisites
This project assumes that you are using Ubuntu (tested on Ubuntu 20.4 LST) and that ROS, gazebo and all the required packages
are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

### Installation
To install the repository follow the following steps:

1. Clone the repository ```$ git clone https://github.com/michailtam/where-am-i.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```

### Running the simulation
To run the simulation follow the following steps:

1. Open a terminal change into the toplevel of the catkin workspace and issue
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
2. Open a second terminal (also change to toplevel) and issue
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```

#### Screenshots
<img src="https://github.com/michailtam/where-am-i/blob/master/images/initial_pose_and_laser_scan.png" alt="initial pose & laser scan" width="760" height="400" border="0" />

