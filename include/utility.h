#pragma once
#ifndef _Param_Server_CLASS_H_
#define _Param_Server_CLASS_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
// PCL
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;
const int rows = 3;
const int cols = 3;
class ParamServer
{
public:
   ros::NodeHandle nh;

   // mode
   bool debugging;
   bool mode;

   // Topics
   string pointCloudTopic;
   string lidarEdgeTopic;
   string lidarSurfTopic;
   string imageTopic;
   string odometryTopic;

   string resultPath;

   // sensor calibration
   vector<double>
       Tr_velo_to_camV;
   vector<double> R0_rectV;
   vector<double> P0V;
   Eigen::MatrixXd Tr_velo_to_cam;
   Eigen::Matrix3d R0_rect;
   Eigen::MatrixXd P0;

   // sensor information
   float scan_period;

   // clustering parameters
   int leaf;
   float object_z_axis_min, object_z_axis_max, object_x_axis_min, object_x_axis_max, object_y_axis_min, object_y_axis_max;
   float object_radius_min, object_radius_max, object_k_merging_threshold, cluster_size_min;

   ParamServer()
   {
      // mode
      nh.param<bool>("VALOAM/debugging", debugging, false);
      nh.param<bool>("VALOAM/mode", mode, false);

      // ros topic list
      nh.param<std::string>("VALOAM/pointCloudTopic", pointCloudTopic, "/points_raw");
      nh.param<std::string>("VALOAM/lidarEdgeTopic", lidarEdgeTopic, "/laser_cloud_edge");
      nh.param<std::string>("VALOAM/lidarSurfTopic", lidarSurfTopic, "/laser_cloud_surf");
      nh.param<std::string>("VALOAM/odometryTopic", odometryTopic, "/odom");
      nh.param<std::string>("VALOAM/imageTopic", imageTopic, "/kitti/camera_gray_left/image_raw");
      nh.param<std::string>("VALOAM/resultPath", resultPath, "/home/taeki/catkin_ws/src/RESULT/ISC");

      std::cout << "pointCloudTopic:" << pointCloudTopic << std::endl;
      std::cout << "lidarEdgeTopic:" << lidarEdgeTopic << std::endl;
      std::cout << "lidarSurfTopic:" << lidarSurfTopic << std::endl;
      std::cout << "imageTopic:" << imageTopic << std::endl;

      // // sensor calibration
      nh.param<vector<double>>("VALOAM/Tr_velo_to_camV", Tr_velo_to_camV, vector<double>());
      nh.param<vector<double>>("VALOAM/R0_rectV", R0_rectV, vector<double>());
      nh.param<vector<double>>("VALOAM/P0V", P0V, vector<double>());
      Tr_velo_to_cam = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(Tr_velo_to_camV.data(), 3, 4);
      R0_rect = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R0_rectV.data(), 3, 3);
      P0 = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(P0V.data(), 3, 4);

      // sensor information
      nh.param<float>("VALOAM/scan_period", scan_period, 0.1);

      // clustering parameters
      nh.param<int>("VALOAM/leaf", leaf, 1);
      nh.param<float>("VALOAM/object_z_axis_min", object_z_axis_min, -50.0);
      nh.param<float>("VALOAM/object_z_axis_max", object_z_axis_max, 50.0);
      nh.param<float>("VALOAM/object_x_axis_min", object_x_axis_min, -50.0);
      nh.param<float>("VALOAM/object_x_axis_max", object_x_axis_max, 50.0);
      nh.param<float>("VALOAM/object_y_axis_min", object_y_axis_min, -50.0);
      nh.param<float>("VALOAM/object_y_axis_max", object_y_axis_max, 50.0);
      nh.param<float>("VALOAM/object_radius_min", object_radius_min, 5.0);
      nh.param<float>("VALOAM/object_radius_max", object_radius_max, 50.0);
      nh.param<float>("VALOAM/object_k_merging_threshold", object_k_merging_threshold, 5.0);
      nh.param<float>("VALOAM/cluster_size_min", cluster_size_min, 20.0);

      usleep(100);
   }
};

#endif //_Param_Server_CLASS_H_