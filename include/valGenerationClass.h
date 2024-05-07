#ifndef _VAL_GENERATION_CLASS_H_
#define _VAL_GENERATION_CLASS_H_

#include "utility.h"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

// opencv
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace Eigen;
typedef std::vector<std::tuple<std::vector<cv::KeyPoint>, cv::Mat, pcl::PointCloud<pcl::PointXYZI>, cv::Mat, Eigen::Isometry3d, ros::Time>> featureMap;
typedef std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZI>>> featurePair;
class VALGENERATIONCLASS : public ParamServer
{
public:
   VALGENERATIONCLASS();
   ~VALGENERATIONCLASS();
   void getEdgeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time);
   void getSurfDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time);
   void planeDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time);
   void print_matchedEdgeFeature(int frame_idx, int matched_idx, int *cnt, pcl::PointCloud<pcl::PointXYZI> *matched_pc, std::vector<int> *map_idx);
   void print_matchedSurfFeature(int frame_idx, int matched_idx, int *cnt, pcl::PointCloud<pcl::PointXYZI> *matched_pc, std::vector<int> *map_idx);
   void planeFeatureMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr &matched_out_curr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &matched_out_prev, std::vector<cv::KeyPoint> *matched_out_key_prev, std::vector<cv::KeyPoint> *matched_out_key_curr);
   void planeFeatureMatch_Plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &matched_out_curr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &matched_out_prev, std::vector<cv::KeyPoint> *matched_out_key_prev, std::vector<cv::KeyPoint> *matched_out_key_curr);
   void edgeFeatureMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_map, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_matched);
   void surfFeatureMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_map, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_matched);
   void downSampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out);
   void updateMap(ros::Time odom_time, Eigen::Isometry3d odom_in);
   int frame_cnt;

private:
   bool init;
   float max_dist;
   featureMap localPlaneMap;
   featureMap globalPlaneMap;
   featureMap localEdgeMap;
   featureMap globalEdgeMap;
   featureMap localSurfMap;
   featureMap globalSurfMap;
   featurePair surfPair;
   featurePair edgePair;
   std::vector<std::vector<int>> matchedEdgeFeature;
   std::vector<std::vector<int>> matchedSurfFeature;
   std::vector<std::vector<int>> matchedPlaneFeature;
   Eigen::Isometry3d odom;
   std::vector<pcl::PointCloud<pcl::PointXYZI>> matchedEdgePC;
   std::vector<pcl::PointCloud<pcl::PointXYZI>> matchedSurfPC;
   // surf points downsampling
   float voxelSize;
   pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
};

#endif // _VAL_GENERATION_CLASS_H_
