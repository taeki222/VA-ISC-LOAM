# VA-LOAM
## Visual Assisted LiDAR Odometry and Mapping for Accurate Autonomous Navigation (VA-LOAM) 

VA-LOAM integrates visual assistance into LiDAR-based odometry and mapping systems, significantly enhancing the accuracy and robustness of SLAM for autonomous navigation." 


## 1. Comparison

![image](https://github.com/taeki222/VA-ISC-LOAM/assets/10024231/73009a4b-a57c-4403-9562-9a0405e43814)


## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.

ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 2.3. **GTSAM**
Follow [GTSAM Installation](https://gtsam.org/get_started/).

### 2.3. **OPENCV**
Follow [OPENCV Installation](https://opencv.org/releases/).


## 3. Build 
### 3.1 Clone repository:
```
cd ~/catkin_ws/src
git clone https://github.com/taeki222/VA-ISC-LOAM.git
cd ..
catkin_make -j1
source ~/catkin_ws/devel/setup.bash
```
### 3.2 Download test rosbag
Data set. [KITTI data set](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

ROS Bag. [KITTI -> ROS Bag](https://github.com/tomas789/kitti2bag)

### 3.3 Launch ROS
```
roslaunch iscloam iscloam_mapping.launch
```

## 4. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated 
```
@inproceedings{,
  author={},
  booktitle={}, 
  title={}, 
  year={},
  volume={},
  number={},
  pages={},
  doi={}
}
```

## 5.Acknowledgements
Thanks for [ISCLOAM](https://github.com/wh200720041/iscloam).
