// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "odomEstimationClass.h"
#include "valGenerationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<sensor_msgs::ImageConstPtr> imgBuf;
cv_bridge::CvImagePtr imageRaw;

lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void imageRawHandler(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_lock.lock();
    imgBuf.push(msg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time = 0;
int total_frame = 0;
void odom_estimation()
{
    VALGENERATIONCLASS VALAM;

    while (1)
    {

        if (!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty() && !pointCloudBuf.empty() && !imgBuf.empty()) //&& !pointCloudEdgeMapBuf.empty() && !pointCloudMatchedEdgeBuf.empty() && !pointCloudSurfMapBuf.empty() && !pointCloudMatchedSurfBuf.empty()        {
        {
            struct timespec begin, end;
            clock_gettime(CLOCK_MONOTONIC, &begin);
            // read data
            mutex_lock.lock();
            if (!pointCloudBuf.empty() && (pointCloudBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || pointCloudBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> odom correction\n");
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;
            }

            if (!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction\n");
                mutex_lock.unlock();
                continue;
            }

            if (!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction \n");
                mutex_lock.unlock();
                continue;
            }
            if (!imgBuf.empty() && (imgBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || imgBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                imgBuf.pop();
                ROS_INFO("imgBuf time stamp unaligned with extra point cloud, pls check your data --> odom correction \n");
                mutex_lock.unlock();
                continue;
            }

            // if time aligned
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());

            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudBuf.pop();

            /*** Convert ROS message to OpenCV ***/
            imageRaw = cv_bridge::toCvCopy(imgBuf.front(), sensor_msgs::image_encodings::BGR8);
            cv::Mat grayScaleImg;
            cv::cvtColor(imageRaw->image, grayScaleImg, CV_BGR2GRAY);
            imgBuf.pop();

            mutex_lock.unlock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
            // VALAM.downSampling(pointcloud_surf_in, downsampledSurfCloud);
            if (VALAM.mode)
            {
                VALAM.getEdgeDescriptor(pointcloud_edge_in, grayScaleImg, pointcloud_time);
                // VALAM.getSurfDescriptor(downsampledSurfCloud, grayScaleImg, pointcloud_time);
                VALAM.frame_cnt++;
            }
            if (is_odom_inited == false)
            {
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_map(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_matched(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_map(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_matched(new pcl::PointCloud<pcl::PointXYZI>());
                if (VALAM.mode)
                    VALAM.edgeFeatureMatch(pointcloud_edge_map, pointcloud_edge_matched);
                // VALAM.surfFeatureMatch(pointcloud_surf_map, pointcloud_surf_matched);

                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, pointcloud_edge_matched, pointcloud_edge_map, pointcloud_surf_matched, pointcloud_surf_map, VALAM.mode);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time += time_temp;
                // ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }
            if (VALAM.mode)
                VALAM.updateMap(pointcloud_time, odomEstimation.odom);
            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            // q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aft_mapped"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "world";
            laserOdometry.child_frame_id = "aft_mapped";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);
            clock_gettime(CLOCK_MONOTONIC, &end);
            std::cerr << "create feature descriptor processing time: " << (end.tv_sec - begin.tv_sec) + ((end.tv_nsec - begin.tv_nsec) / 1000000000.0) << std::endl;
        }
        // sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period = 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;

    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subImageRaw = it.subscribe("/kitti/camera_gray_left/image_raw", 1, imageRawHandler);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
