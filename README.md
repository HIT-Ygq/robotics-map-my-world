# Robotics Simulation - Where Am I 

In this robotics simulation in [gazebo](http://gazebosim.org/) a custom *mobile robot* applies the Adaptive Monte Carlo Localization algorithm (AMCL), to determine it's position and pose in the environment (i.e. world). The (AMCL ROS package)[http://wiki.ros.org/amcl] improves the localization process of the MCL algorithm, due to dynamically adjusting the number of particles over a period of time, as the robot navigates in the map. In addition, the robot applies laser scanning ([Lidar](https://en.wikipedia.org/wiki/Lidar) to get data for mapping the environment. The ROS version that gets used in the project is [ROS-Kinetic](https://www.ros.org/). The 3D visualization tool [Rviz tool for ROS](http://wiki.ros.org/rviz) gets used to ... (to be continued) 

### Features
- An adaptive version (AMCL) of the Monte Carlo Localization [MCL](https://en.wikipedia.org/wiki/Monte_Carlo_localization) algorithm gets applied, which dynamically adjusts the number of particles over a period of time, as the robot navigates in the a environment. This provides a significant computational advantage over the MCL algorithm. In other words, AMCL gets used to take odometry and laser scan data to perform the AMCL localization.
- A laser scaner gets applied to map the environment (i.e. building), which helps the robot to determine its surroundings.
- The ROS (Map Server node)[http://wiki.ros.org/map_server)] provides map data as a ROS service to other nodes and locates the map which gets created by the [ROS Map Creator](https://github.com/udacity/pgm_map_creator). This map gets used in Rviz to map the environment of gazebo.
- The [ROS move_base](http://wiki.ros.org/move_base) and [ROS teleop](http://wiki.ros.org/teleop_twist_keyboard) packages are used to define a navigation goal position for the robot, so the robot will navigate to that goal position.


### Prerequisites
This project assumes that you are using Ubuntu (tested on Ubuntu 20.4 LST) and that ROS, gazebo and all the required packages
are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

### Installation
To install the repository and packages follow the following steps:

1. Clone the repository ```$ git clone https://github.com/michailtam/where-am-i.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```
4. You also need to install the following packages:
```
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ sudo apt-get install ros-kinetic-amcl
```

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
The AMCL algorithm gets applied using the 3D Visualization tool Rviz. The below screenshots show how the AMCL algorithm
and the laser scan get used so the robot is able to determine it's location in the environment. 

| **Initial position and pose** |
| :--- |
| At startup the robot gets placed at the center of the building ***(0,0)***. The blue flat squares show how the laser scan and the walls coincide together. This is necessary, because otherwise we do not know if the created map matches the dimensions of the environment in gazebo. This would lead to the inaccuracy of the localization process. |
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/initial_pose_and_laser_scan.png" alt="Initial pose & laser scan" width="560" height="300" border="0" /> |

| **Different initial start pos** |
| :--- |
| The robot gets placed at another place in the building ***(left side of the floor)***, but its orientation keeps the same. In this case, the laser scan does not coincide with the walls and the particles are spread nearly over the entire building. This shows that the robot at this state does not know it's location ***(position and orientation)*** in relation to the building. |
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/other_initial_start_pos.png" alt="Different initial start pos" width="560" height="300" border="0" /> |

| **1st movement observation** |
| :--- | 
| This observation shows that the particles start to converge towards the robot ***(the particles near the robot are getting more dense)*** and the laser scan gets adjusted towards the walls. This is expected, because while the particles are getting updated, the robot gets more and more sure about it's location in the environment. | 
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_01.png" alt="1st AMCL update" width="560" height="300" border="0" /> |

| **2nd movement observation** |
| :--- |
| This observation shows that the particles have better converged ***(as before)*** toward the robot, but there are still a lot of particles which do not coincide with the robots location. Also, the screenshot shows that the laser scan nearly matches the walls. | 
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_02.png" alt="2nd AMCL update" width="560" height="300" border="0" /> |

| **3rd movement observation** |
| :--- |
| This observation shows that the particles have converged toward the robot and there are only some particles remaininig which do not coincide with the robots location. The laser scan matches matches entirely the walls. |  
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_03.png" alt="3rd AMCL update" width="560" height="300" border="0" /> |

| **4th movement observation** | 
| :--- |
| This observation shows that the particles have completely converged toward the robot and there are less particles which do not coincide with the robots location. The screenshot shows that the robot avoids to crash towards the wall, which proves that the robot knows entirely it's location in the building. The laser scan matches entirely the walls as previously. |  
| **Screenshot** |
| <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_04.png" alt="4th AMCL update" width="560" height="300" border="0" /> |



