
# NDT Stanley Controller 


- [NDT Stanley Controller ](#ndt-stanley-controller)
  - [Description](#purpose)
    - [Host PC Configuration](#host-pc-configuration)
  - [How to Run](#how-to-run)
    - [1. Create a catkin workspace](#1-create-a-catkin-workspace)
    - [2. Build the package](#3-build-the-package)
    - [3. Run the package](#4-run-the-package)



### Description

This repository is used to localize the vehicle using the lidar point cloud and ros bag file.




#### Dependencies

|                          |              |                                                                         |
|--------------------------|--------------| ----------------------------------------------------------------------- |
| Host PC Software Version | Ubuntu 20.04 |        https://releases.ubuntu.com/focal/                               |
| Python Version           | 3.8.10       | https://www.python.org/downloads/release/python-3810/                   |
| Ros Version              | Ros 1 Noetic | https://wiki.ros.org/noetic   |


### How to Run

#### 1. Create a catkin workspace

```bash
mkdir -p ~/ndt_localization_ws/src
cd  ndt_stanley_controller_ws/src
cd ..
catkin_make
source devel/setup.bash
```

>**Note:**
> If you want to use the ros bag file to test the code, you have to declare the topic name in the code same as the topic name that publish in the ros bag file.
> In this case, the topic name is **/velodyne_points**. 

#### 2. Build the package

```bash
cd ~/.../ndt_stanley_controller_ws
catkin_make
source devel/setup.bash
```
#### 3. Run the package

```bash
cd  ~/.../ndt_stanley_controller_ws/src/ndt_stanley_controller
rosrun ndt_stanley_controller ndt_stanley_controller_node
```
for play ndt localizer you should to run command given below in different terminal
```bash
cd ../ndt_localization_ws
source devel/setup.bash
roslauch roslaunch lidar_localizer ndt_mapping.launch 
```


for play rosbag record you should to run command given below in different terminal
```bash
rosbag play hdl_map.bag --clock
```
for listen the topic named **tf** run the following command in different terminal

```bash
rostopic echo /tf
```

