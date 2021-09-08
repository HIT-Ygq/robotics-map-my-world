# Robotics Simulation - Map My World
In this robotics simulation a custom *mobile robot* moves in a building and executes [*Simultaneous-Localization-And-Mapping - SLAM*](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) to determine its environment where it is acting. SLAM is a combination of localization which means where the robot is located with respect to the global frame (i.e. x and y position and it's orientation) and mapping is a method to create a 2D or 3D map of the unknown environment. One method to perform localization is shown in the project [where am i](https://github.com/michailtam/where-am-i), where a particle filter gets used to locate the robot in the building. In order, for the robot to be capable to map its environment, it has to be equiped with special kind of sensors. One way to do this is to use a [Lidar sensor](https://en.wikipedia.org/wiki/Lidar) which reflects the rays from the walls to calculate the distance to obstacles. The second way is to use an [RGB-D camera](https://en.wikipedia.org/wiki/Kinect) which is comprised of a a single visual camera combined with a laser rangefinder or infrared depth sensor, and allows for the determination of the depth of an image (i.e. distance to a wall). Please, refer to the following link to get more information about [range imaging](https://en.wikipedia.org/wiki/Range_imaging). Unfortunately, reading sensor data tends to be noisy, because of a fault or a bad calibrated sensor. So a better way is to perform sensor fusion, which means that multiple data from different sensors get combined together to achieve a better result with less noise. The simulation gets executed in [gazebo](http://gazebosim.org/) using [ROS-Noetic](https://www.ros.org/). To perform 3D SLAM the [ROS RTAB-Map package](http://wiki.ros.org/rtabmap_ros) gets used which implements a RGB-D SLAM approach with real-time constraints. To track the operation visually the visualization tool [Rviz tool for ROS](http://wiki.ros.org/rviz) will be used which depicts the resulted map in 3D.

### Features
- To get the data from the laser scanner the robot subsrcibes to the **/scan topic**
- The supported laser scanners in ROS are [these](http://wiki.ros.org/Sensors#A2D_range_finders) here
- RGB-D SLAM with real-time constraints
- RTAB-DB Viewer which allows for complete analysis of the mapping session

### Prerequisites
This project assumes that you are using Ubuntu (tested on Ubuntu 20.4 LST) and that ROS (tested on noetic), gazebo and all the required packages are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

### Installation
To install the repository and packages, please follow the bellow steps. If you encounter any problems please refer to the [disussion forum of ROS](https://discourse.ros.org/) to get further help.

1. Clone the repository ```$ git clone https://github.com/michailtam/map-my-world.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```
4. You also need to install the following packages:
```
$ git clone -b noetic-devel https://github.com/ros-planning/navigation.git
```

### Running the simulation:
To run the simulation follow the following steps:

Open a terminal, change into the toplevel of the catkin_workspace and issue: 
```./test_mapping.sh```

Alternatively you can execute each script seperately with the following steps:
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

### Rviz
Rviz is pre-configured with default parameters to execute SLAM. Feel free to adjust these parameters to your needs.

### RTAB-DB Viewer:
The [**rtabmap-databaseViewer**](https://github.com/introlab/rtabmap/wiki/Tools) is a tool to explore the created database, when the mapping process has completed. It is isolated from ROS which allows the complete analysis the mapping session. This makes it possible to check for loop closures, generate 3D maps, extract images, and much more!. To run the RTAB-DB Viewer issue 
```$ rtabmap-databaseViewer ~/.ros/rtabmap.db```.
Once the viewer is opened, you need to execute the following steps to add some windows to view relevant information:

Choose Yes for using the database parameters
View -> Constraint View
View -> Graph View

#### Screenshots
| **1st RTAB-DB Output** |
| :--- |
| Left: The created 2D map and the path the robot is traversing, Center: The output of the camera, Right: The constraints view | 
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start_0.png" alt="Map after SLAM" width="660" height="400" border="0" /> |

| **2nd RTAB-DB Output** |
| :--- |
| Left: The created 2D map and the path the robot is traversing, Center: The output of the camera, Right: The constraints view |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start.png" alt="DB viewer while after SLAM" width="660" height="400" border="0" /> |

| **3D Office in Gazebo** |
| :--- |
| The office where the robot is moving and performs SLAM |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-gazebo.png" alt="Gazebo office" width="660" height="400" border="0" /> |

| **SLAM visualization in Rviz (in 3D)** |
| :--- |
| The visualized output in rviz after performing SLAM |
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-rviz.png" alt="SLAM in rviz" width="660" height="400" border="0" /> |

| **Created 2D Map after SLAM** | **Created 3D Map after SLAM** |
| :---: | :---: | 
| The created 2D office map | The created 3D office map |
| **Screenshots** | **Screenshots** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/slam_rviz_01.png" alt="Created 2D map" width="360" height="600" border="0" /> | <img src="https://github.com/michailtam/map-my-world/blob/master/images/slam_rviz_02.png" alt="Created 3D map" width="360" height="550" border="0" /> |

An example of executing SLAM is shown in the video below.

#### Video
<a href="https://youtu.be/6FNHveEkFfM" target="_blank">
<img src="https://github.com/michailtam/map-my-world/blob/master/images/video_preview.png" alt="Map My World (ROS) Video" width="560" height="300" border="0" />