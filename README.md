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

| Description | Screenshot |
| --- | --- |
| **Initial position and pose**<br/>At startup the robot gets placed at the center of the building (0,0). The blue lines show that the laser scaner and tha walls coincide properly. | <img src="https://github.com/michailtam/where-am-i/blob/master/images/initial_pose_and_laser_scan.png" alt="Initial pose & laser scan" width="760" height="400" border="0" /> |
| Different initial start pos | <img src="https://github.com/michailtam/where-am-i/blob/master/images/other_initial_start_pos.png" alt="Different initial start pos" width="760" height="400" border="0" /> |
| 1st AMCL update | <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_01.png" alt="1st AMCL update" width="760" height="400" border="0" /> |
| 2nd AMCL update | <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_02.png" alt="2nd AMCL update" width="760" height="400" border="0" /> |
| 3rd AMCL update | <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_03.png" alt="3rd AMCL update" width="760" height="400" border="0" /> |
| 4th AMCL update | <img src="https://github.com/michailtam/where-am-i/blob/master/images/movement_amcl_update_04.png" alt="4th AMCL update" width="760" height="400" border="0" /> |


<table>
    <th>Description</th><th>Screenshot</th>
    <tr>
        <td><b>Initial position and pose</b><br/>At startup the robot gets placed at the center of the building (0,0). The blue lines show that the 		laser scaner and tha walls coincide properly.</td>
        <td><img src="https://github.com/michailtam/where-am-i/blob/master/images/initial_pose_and_laser_scan.png" alt="Initial pose & laser scan" 		width="660" height="300" border="0" /></td>
    </tr>
</table>









