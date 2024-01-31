
# Iterative Close Points Localization


- [Iterative Close Points Localization](#Iterative-Close-Points-Localization)
  - [Purpose](#purpose)
    - [Host PC Configuration](#host-pc-configuration)
  - [Inputs](#inputs)
  - [Outputs](#outputs)
  - [Implementation](#implementation)
    - [1. Create a catkin workspace](#1-create-a-catkin-workspace)
    - [2. Clone the repository](#2-clone-the-repository)
    - [3. Build the package](#3-build-the-package)
    - [4. Run the package](#4-run-the-package)
    - [5. Result](#5-result)


### Purpose

This repository is used to localize the vehicle using the lidar point cloud and ros bag file.


|                         |                                                 | 
|-------------------------|-------------------------------------------------|
| ICP Localization Github | https://github.com/cwchen1102/icp_localization |


#### Host PC Configuration

|                          |              |                                                                         |
|--------------------------|--------------| ----------------------------------------------------------------------- |
| Host PC Software Version | Ubuntu 20.04 |        https://releases.ubuntu.com/focal/                               |
| Python Version           | 3.8.10       | https://www.python.org/downloads/release/python-3810/                   |
| Ros Version              | Ros 1 Noetic | https://wiki.ros.org/noetic   |




### Inputs

|          |                            | 
|----------|----------------------------|
| PCD file | Clean and cropped map file |
| Bag file | ros1 bag record            |

### Outputs

- **scan** : The **scan** topic is where data collected and processed by a sensor is published. In this case, the **scan** topic published with scan_pub.publish(Final_RGB); contains the point cloud acquired by the sensor and aligned using Iterative Closest Point (ICP). This aligned point cloud is commonly used in mapping and localization applications.
- **tf** :  The **tf** topic is where transformation information (such as sensor position, map position, etc.) is published. The **tf** topic published with tf_brocast(trans); specifies the position and/or orientation of the sensor. This position and orientation information is often used to convert and align the sensor's physical position to map coordinates.
- **map** : The **map** topic is where map data is published. The 'map' topic published with map_pub.publish(cloud_RGB); typically contains a map generated based on environmental data perceived by the sensor. This map serves as a representation of the environment sensed by the sensor and is commonly used in navigation and localization systems.
- **Rotation Submatrix** : This matrix represents the transformation of an object in 3D space. Each row and column specifies the amount of rotation around the three-dimensional coordinate axes, which are perpendicular to each other. For instance, cell (1, 1) indicates the rotation around the x-axis, cell (2, 1) represents the rotation around the y-axis, and cell (3, 1) denotes the rotation around the z-axis. This submatrix defines the rotation required to align a point cloud to another in the context of the code.
- **Translition Companents** : These components define the position of an object in 3D space. Typically represented as a three-dimensional vector (Tx, Ty, Tz), they specify the amount of displacement required along each axis to align one point cloud to another. For example, the Tx component represents displacement along the x-axis, Ty along the y-axis, and Tz along the z-axis. These components enable an object to be positioned at a specific location in space.

### Implementation

#### 1. Create a catkin workspace

```bash

mkdir -p ~/ICP_Localization_ws/src
cd ~/ICP_Localization_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash

```
#### 2. Clone the repository

```bash

cd ~/Desktop/fh_workspace/localization/ICP_Localization_ws/src
git clone https://github.com/cwchen1102/icp_localization.git

```
>**Note:** 
> If you want to use the ros bag file to test the code, you have to declare the topic name in the code same as the topic name that publish in the ros bag file.
> In this case, the topic name is /velodyne_points. 

#### 3. Build the package

```bash

cd ~/Desktop/fh_workspace/localization/ICP_Localization_ws
catkin_make
source devel/setup.bash

```
#### 4. Run the package

```bash

cd ~/ICP_Localization_ws/src/icp_localization
rosrun hw5_0751081 hw5_0751081_node

```
#### 5. Result

in different terminal 

```bash
cd /mnt/ce539d25-c534-4d26-8907-788ffe0089fb/280124_hosab2/hosab2_280124_rosbag

rosbag play hdl_map.bag --clock

```
for listen the topic named **tf** run the following command in different terminal

```bash
source ~/Desktop/fh_workspace/localization/ICP_Localization_ws/devel/setup.bash

rostopic echo /tf

```




