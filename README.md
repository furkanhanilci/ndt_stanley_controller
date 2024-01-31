
# Iterative Close Points Localization

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



### Dependencies

|          |                            | 
|----------|----------------------------|
| PCD file | Clean and cropped map file |
| Bag file | ros1 bag record            |

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




