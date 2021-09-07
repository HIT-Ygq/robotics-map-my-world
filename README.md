# Robotics Simulation - Map My World
In this robotics simulation a custom *mobile robot* moves in a building and executes *SLAM* (Simultaneous-Localization-And-Mapping) to determine its environment where it acts. SLAM is a combination of localization which means where the robot is located (i.e. x and y position and it's orientation) and mapping is a method to create a reflection of the unknown environment in either 2D or 3D. One method to perform localization is shown in the project [where am i](https://github.com/michailtam/where-am-i) where so called particles are used to locate the robot in the building. In order, for the robot to be capable to map its environment, it needs to be equiped with special sensors. One way to do this is to use a [Lidar sensor](https://en.wikipedia.org/wiki/Lidar) which reflects the rays from the walls to calculate the distances to the obstacles. The second way is to use an [RGB-D camera](https://en.wikipedia.org/wiki/Kinect) which is a single visual camera combined with a laser rangefinder or infrared depth sensor, and allows for the determination of the depth of an image (i.e. distance to a wall). Please, refer to the following link to get more information about [range imaging](https://en.wikipedia.org/wiki/Range_imaging). Because, every sensor has a tend to read noisy data, combining multiple sensors is a better way to perform mapping. So, the robot combines both sensor data to get better results while it maps its envirnoment. The simulation gets executed in [gazebo](http://gazebosim.org/) using [ROS-Noetic](https://www.ros.org/). To perform 3D SLAM the [ROS RTAB-Map package](http://wiki.ros.org/rtabmap_ros) gets used which implements a RGB-D SLAM approach with real-time constraints. To track the operation visually the 3D visualization tool [Rviz tool for ROS](http://wiki.ros.org/rviz) is used which shows the created map in 3D.

### Features
- To get the data from the laser scanner the robot subsrcibes to the **/scan topic**
- The supported laser scanners in ROS are [these](http://wiki.ros.org/Sensors#A2D_range_finders) here
- RGB-D SLAM with real-time constraints

### Prerequisites
This project assumes that you are using Ubuntu (tested on Ubuntu 20.4 LST) and that ROS (tested on noetic), gazebo and all the required packages are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

### Installation
To install the repository and packages follow the following steps:

1. Clone the repository ```$ git clone https://github.com/michailtam/map-my-world.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```
4. You also need to install the following packages:
```
$ git clone -b noetic-devel https://github.com/ros-planning/navigation.git
```

### Running the simulationTo run the simulation follow the following steps:
To run the simulation follow the following steps:

1. Open a terminal change into the toplevel of the catkin workspace and issue
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
2. Open a second terminal (also change to toplevel) and issue
```
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

In Rviz setup the following ... (IS COMING SOON)

#### Screenshots
| **2D Map after SLAM** |
| :--- |
| The created 2D map and path after SLAM and what the camera sees |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start_0.png" alt="Map after SLAM" width="660" height="400" border="0" /> |

| **Generated particles while executing SLAM (DB Viewer)** |
| :--- |
| The created 2D map and output of the DB viewer (particles) |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start.png" alt="DB viewer while after SLAM" width="660" height="400" border="0" /> |

| **3D Office in Gazebo** |
| :--- |
| The office where the robot performs SLAM |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-gazebo.png" alt="Gazebo office" width="660" height="400" border="0" /> |

| **SLAM visualized in 3D in Rviz** |
| :--- |
| The visualized output in rviz after performing SLAM |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-rviz.png" alt="SLAM in rviz" width="660" height="400" border="0" /> |

| **Created 2D Map after SLAM** | **Created 3D Map after SLAM** |
| :---: | :---: | 
| The created 2D office map | The created 3D office map |
| **Screenshots** | **Screenshots** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/slam_rviz_01.png" alt="Created 2D map" width="360" height="600" border="0" /> | <img src="https://github.com/michailtam/map-my-world/blob/master/images/slam_rviz_02.png" alt="Created 3D map" width="360" height="600" border="0" /> |




