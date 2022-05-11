# A-LOAM
## Advanced implementation of LOAM

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. It is a good learning material for SLAM beginners.

<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/>

**Modifier:** [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```

## 4. DURABLE Usage
### 4.1 Record a rosbag around the mapping area
Drive around the area to map recording at least the pointcloud from the velodyne (/velodyne_points). It is suggested the robot starts this bag facing East. While you are driving the robot try to have the world static and be sure to be close to the robot so your presence can be filtered from the map.

### 4.2 Tune Parameters
Tune the parameters in the launch file *durable.launch*, including the minimum distance to create the map (the max distance you were from the robot).

### 4.3 Launch A-Loam
To generate the map open two terminals, in the first one launch the a-loam algorithm:

```
    roslaunch aloam_velodyne durable.launch
```

In the second one run the bag you recorded to generate the map:

```
    rosbag play YOUR_DATASET_FOLDER/YOUR_BAG.bag
```
### 4.3 Save the data
The map is published to a topic /laser_cloud_map so, in order to save it, we will record the last seconds of the mapping. When the mapping is close to be completed (the bag you are playing is 10-15 seconds away from finishing) record the map and other topics by running:
```
    roslaunch aloam_velodyne record_result.launch
```
This will save the result of a-loam in a rosbag file.

**NOTE:** - You should configure the path where the rosbag will be recorded inside **record_result.launch**.

## 5.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

