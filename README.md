
# Iterative Close Points Localization


- [Iterative Close Points Localization](#Iterative-Close-Points-Localization)
  - [Description](#purpose)
    - [Host PC Configuration](#host-pc-configuration)
  - [How to Run](#How to Run)
    - [1. Create a catkin workspace](#1-create-a-catkin-workspace)
    - [2. Clone the repository](#2-clone-the-repository)
    - [3. Build the package](#3-build-the-package)
    - [4. Run the package](#4-run-the-package)
    - [5. Result](#5-result)


### Description

This repository is used to localize the vehicle using the lidar point cloud and ros bag file.


|                         |                                                                           | 
|-------------------------|---------------------------------------------------------------------------|
| ICP Localization Github | https://github.com/furkanhanilci/iterative_close_points_localization.git  |


#### Dependencies

|                          |              |                                                                         |
|--------------------------|--------------| ----------------------------------------------------------------------- |
| Host PC Software Version | Ubuntu 20.04 |        https://releases.ubuntu.com/focal/                               |
| Python Version           | 3.8.10       | https://www.python.org/downloads/release/python-3810/                   |
| Ros Version              | Ros 1 Noetic | https://wiki.ros.org/noetic   |



### How to Run

#### 1. Create a catkin workspace

```bash
mkdir -p ~/icp_localization_ws/src
cd  icp_localization_ws/src
cd ..
catkin_make
source devel/setup.bash
```

#### 2. Clone the repository
```bash
cd  ~/.../icp_localization_ws/src
git clone https://github.com/cwchen1102/icp_localization.git
```

>**Note:**
> If you want to use the ros bag file to test the code, you have to declare the topic name in the code same as the topic name that publish in the ros bag file.
> In this case, the topic name is **/velodyne_points**.

#### 3. Build the package

```bash
cd ~/.../icp_localization_ws
catkin_make
source devel/setup.bash
```
#### 4. Run the package

```bash
cd  ~/.../icp_localization_ws/src/icp_localization
rosrun hw5_0751081 hw5_0751081_node
```
for play rosbag record you should to run command given below in different terminal
```bash
rosbag play hdl_map.bag --clock
```
for listen the topic named **tf** run the following command in different terminal

```bash
rostopic echo /tf
```




