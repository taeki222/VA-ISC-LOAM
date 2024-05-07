#include "valGenerationClass.h"
VALGENERATIONCLASS::VALGENERATIONCLASS()
{
   init = false;
   frame_cnt = 0;
   max_dist = 5.0;
   voxelSize = 0.6;
   downSizeFilterSurf.setLeafSize(voxelSize, voxelSize, voxelSize);
}
VALGENERATIONCLASS::~VALGENERATIONCLASS()
{
}
void VALGENERATIONCLASS::print_matchedEdgeFeature(int frame_idx, int matched_idx, int *cnt, pcl::PointCloud<pcl::PointXYZI> *matched_pc, std::vector<int> *map_idx)
{
   if (frame_idx < 0 || matched_idx < 0)
      return;
   if (debugging)
      fprintf(stderr, "%f %f %f;\n", matchedEdgePC[frame_idx][matched_idx].x, matchedEdgePC[frame_idx][matched_idx].y, matchedEdgePC[frame_idx][matched_idx].z);

   matched_pc->push_back(matchedEdgePC[frame_idx][matched_idx]);
   map_idx->push_back(frame_idx);
   if (matchedEdgeFeature[frame_idx][matched_idx] >= 0)
   {
      int prev_idx = matchedEdgeFeature[frame_idx][matched_idx];
      *cnt = *cnt + 1;
      print_matchedEdgeFeature(frame_idx - 1, prev_idx, cnt, matched_pc, map_idx);
   }
   else
      return;
}
void VALGENERATIONCLASS::downSampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out)
{
   downSizeFilterSurf.setInputCloud(surf_pc_in);
   downSizeFilterSurf.filter(*surf_pc_out);
}
void VALGENERATIONCLASS::print_matchedSurfFeature(int frame_idx, int matched_idx, int *cnt, pcl::PointCloud<pcl::PointXYZI> *matched_pc, std::vector<int> *map_idx)
{
   if (frame_idx < 0 || matched_idx < 0)
      return;
   // fprintf(stderr, "%f %f %f;\n", matchedEdgePC[frame_idx][matched_idx].x, matchedEdgePC[frame_idx][matched_idx].y, matchedEdgePC[frame_idx][matched_idx].z);

   matched_pc->push_back(matchedSurfPC[frame_idx][matched_idx]);
   map_idx->push_back(frame_idx);
   if (matchedSurfFeature[frame_idx][matched_idx] >= 0)
   {
      int prev_idx = matchedSurfFeature[frame_idx][matched_idx];
      *cnt = *cnt + 1;
      print_matchedSurfFeature(frame_idx - 1, prev_idx, cnt, matched_pc, map_idx);
   }
   else
      return;
}
void VALGENERATIONCLASS::surfFeatureMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_map, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_matched)
{

   surfPair.clear();
   if (globalSurfMap.size() > 0 && localSurfMap.size() > 0)
   {
      fprintf(stderr, "globalSurfMap thread\n");

      /*ORB를 위한 FLANN 설정값*/
      const static auto indexParams = new cv::flann::IndexParams();
      indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
      indexParams->setInt("table_number", 6);
      indexParams->setInt("key_size", 12);
      indexParams->setInt("multi_probe_level", 1);

      const static auto searchParams = new cv::flann::SearchParams();
      searchParams->setInt("checks", 50);

      /*match features between two surf*/
      int localMapSize = (int)localSurfMap.size();
      for (int i = 0; i < localMapSize; i++)
      {

         // local data
         cv::Mat descriptor_curr = std::get<1>(localSurfMap[i]);
         pcl::PointCloud<pcl::PointXYZI> pc_curr = std::get<2>(localSurfMap[i]);
         int globalMapSize = (int)globalSurfMap.size();
         for (int j = globalMapSize - 1; j < globalMapSize; j++)
         {
            // map data
            cv::Mat descriptor_prev = std::get<1>(globalSurfMap[j]);
            pcl::PointCloud<pcl::PointXYZI> pc_prev = std::get<2>(globalSurfMap[j]);

            // matching based flann
            const static auto flann = cv::FlannBasedMatcher(indexParams, searchParams);
            std::vector<std::vector<cv::DMatch>> matches;
            flann.knnMatch(descriptor_prev, descriptor_curr, matches, 2);
            const float ratio_thresh = 0.50f;
            std::vector<cv::DMatch> good;
            good.clear();
            for (const auto &mn : matches)
            {
               if (mn.size() < 2)
                  continue;
               if (mn[0].distance < ratio_thresh * mn[1].distance)
               {
                  good.push_back(mn[0]);
                  matchedSurfFeature[frame_cnt - 1][mn[0].trainIdx] = mn[0].queryIdx;
               }
            }
            // draw matching result
            if (debugging && good.size() > 0)
            {
               std::vector<cv::KeyPoint> keypoints_curr = std::get<0>(localSurfMap[i]);
               cv::Mat img_curr = std::get<3>(localSurfMap[i]);
               std::vector<cv::KeyPoint> keypoints_prev = std::get<0>(globalSurfMap[j]);
               cv::Mat img_prev = std::get<3>(globalSurfMap[j]);
               cv::Mat resultMat;
               drawMatches(img_prev, keypoints_prev, img_curr, keypoints_curr, good, resultMat, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
               char windowname[250];
               sprintf(windowname, "surf curr[%d]-prev[%d]", i, j);
               cv::imshow(windowname, resultMat);
               cv::waitKey(1);
            }
         }
         pcl::PointCloud<pcl::PointXYZI> curr_frame_pc;
         pcl::PointCloud<pcl::PointXYZI> matched_frame_pc;

         int sliding_surf_cnt = 0;
         std::vector<pcl::PointCloud<pcl::PointXYZI>> surf_pair;
         std::vector<std::vector<int>> map_idx;
         int matchedFeatureSize = (int)matchedSurfFeature[frame_cnt - 1].size();
         for (int i = 0; i < matchedFeatureSize; i++)
         {
            if (matchedSurfFeature[frame_cnt - 1][i] > -1)
            {
               pcl::PointCloud<pcl::PointXYZI> matched_pc;
               std::vector<int> map_idx_tmp;
               int cnt = 1;

               // fprintf(stderr, "matched_surf=[\n");
               // fprintf(stderr, "%f %f %f;\n", pc_curr[i].x, pc_curr[i].y, pc_curr[i].z);
               matched_pc.push_back(pc_curr[i]);
               map_idx_tmp.push_back(frame_cnt - 1);
               print_matchedSurfFeature(frame_cnt - 1 - 1, matchedSurfFeature[frame_cnt - 1][i], &cnt, &matched_pc, &map_idx_tmp);
               // fprintf(stderr, "];\n");
               if (cnt > 0)
               {
                  sliding_surf_cnt++;
                  surf_pair.push_back(matched_pc);
                  map_idx.push_back(map_idx_tmp);
               }
               pcl::PointXYZI temp_pc;
               int matchedPCSize = (int)matched_pc.size();
               for (int k = 0; k < matchedPCSize; k++)
               {
                  if (k == 0)
                     pc_out_matched->push_back(matched_pc[k]);
                  else
                  {
                     temp_pc.x += matched_pc[k].x;
                     temp_pc.y += matched_pc[k].y;
                     temp_pc.z += matched_pc[k].z;
                     temp_pc.intensity = matched_pc[k].intensity;
                  }
               }
               temp_pc.x /= (matchedPCSize - 1);
               temp_pc.y /= (matchedPCSize - 1);
               temp_pc.z /= (matchedPCSize - 1);
               pc_out_map->push_back(temp_pc);
            }
         }
         // for (int k = 0; k < (int)matched_frame_pc.size(); k++)
         // {
         //    fprintf(stderr, "curr_%d:[%f %f %f]\n", k, curr_frame_pc[k].x, curr_frame_pc[k].y, curr_frame_pc[k].z);
         //    fprintf(stderr, "matc_%d:[%f %f %f]\n", k, matched_frame_pc[k].x, matched_frame_pc[k].y, matched_frame_pc[k].z);
         // }

         fprintf(stderr, "----------sliding surf--[%d][%d]----------\n", frame_cnt, sliding_surf_cnt);
         // fprintf(stderr, "start sliding surf test ----[%d]---- \n", surf_pair.size());
         max_dist = max_dist * 2;
         double dist_th = 0;
         int surfPairSize = (int)surf_pair.size();
         for (int i = 0; i < surfPairSize; i++)
         {
            // fprintf(stderr, "matched_surf=[\n");
            if (surf_pair[i].size() > 2)
            {
               Eigen::Vector3d dt;
               dt << surf_pair[i][1].x - surf_pair[i][2].x, surf_pair[i][1].y - surf_pair[i][2].y, surf_pair[i][1].z - surf_pair[i][2].z;
               // for (int j = 0; j < surf_pair[i].size(); j++)
               // {
               //   fprintf(stderr, "%f %f %f;\n", surf_pair[i][j].x, surf_pair[i][j].y, surf_pair[i][j].z);
               // }
               // fprintf(stderr, "];\n");
               double dist = sqrt(dt(0) * dt(0) + dt(1) * dt(1) + dt(2) * dt(2));
               // fprintf(stderr, "dist: %f\n", sqrt(dt(0) * dt(0) + dt(1) * dt(1) + dt(2) * dt(2)));
               if (dist < 0.4)
               {
                  Eigen::Vector3d dt1;
                  dt1 << surf_pair[i][0].x - surf_pair[i][2].x, surf_pair[i][0].y - surf_pair[i][2].y, surf_pair[i][0].z - surf_pair[i][2].z;
                  Eigen::Vector3d dt2;
                  dt2 << surf_pair[i][0].x - surf_pair[i][1].x, surf_pair[i][0].y - surf_pair[i][1].y, surf_pair[i][0].z - surf_pair[i][1].z;
                  double dist1 = sqrt(dt1(0) * dt1(0) + dt1(1) * dt1(1) + dt1(2) * dt1(2));
                  double dist2 = sqrt(dt2(0) * dt2(0) + dt2(1) * dt2(1) + dt2(2) * dt2(2));
                  if (dist1 < dist2)
                  {
                     if (dist_th < dist2)
                        dist_th = dist2;
                  }
                  else
                  {
                     if (dist_th < dist1)
                        dist_th = dist1;
                  }
               }
            }
         }
         if (dist_th > 1.0)
         {
            max_dist = dist_th;
         }

         fprintf(stderr, "max dist: %f\n", max_dist);
         // fprintf(stderr, "total_surf_num: %d\n", total_surf_num);
         fprintf(stderr, "end sliding surf test -------- \n");
      }

      if (debugging)
         fprintf(stderr, "--------------surf---------------------------\n");
   }
}
void VALGENERATIONCLASS::edgeFeatureMatch(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_map, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_matched)
{
   edgePair.clear();
   if (globalEdgeMap.size() > 0 && localEdgeMap.size() > 0)
   {
      fprintf(stderr, "globalEdgeMap thread\n");

      /*ORB를 위한 FLANN 설정값*/
      const static auto indexParams = new cv::flann::IndexParams();
      indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
      indexParams->setInt("table_number", 6);
      indexParams->setInt("key_size", 12);
      indexParams->setInt("multi_probe_level", 1);

      const static auto searchParams = new cv::flann::SearchParams();
      searchParams->setInt("checks", 50);
      /*match features between two edge*/
      int localMapSize = (int)localEdgeMap.size();
      for (int i = 0; i < localMapSize; i++)
      {
         // local data
         cv::Mat descriptor_curr = std::get<1>(localEdgeMap[i]);
         pcl::PointCloud<pcl::PointXYZI> pc_curr = std::get<2>(localEdgeMap[i]);
         int globalMapSize = (int)globalEdgeMap.size();
         for (int j = globalMapSize - 1; j < globalMapSize; j++)
         {
            // map data
            cv::Mat descriptor_prev = std::get<1>(globalEdgeMap[j]);
            pcl::PointCloud<pcl::PointXYZI> pc_prev = std::get<2>(globalEdgeMap[j]);

            // matching based flann
            const static auto flann = cv::FlannBasedMatcher(indexParams, searchParams);
            std::vector<std::vector<cv::DMatch>> matches;
            flann.knnMatch(descriptor_prev, descriptor_curr, matches, 2);
            const float ratio_thresh = 0.40f;
            std::vector<cv::DMatch> good;
            good.clear();
            for (const auto &mn : matches)
            {
               if (mn.size() < 2)
                  continue;
               if (mn[0].distance < ratio_thresh * mn[1].distance)
               {
                  good.push_back(mn[0]);
                  matchedEdgeFeature[frame_cnt - 1][mn[0].trainIdx] = mn[0].queryIdx;
               }
            }
            // draw matching result
            if (debugging && good.size() > 0)
            {
               std::vector<cv::KeyPoint> keypoints_curr = std::get<0>(localEdgeMap[i]);
               cv::Mat img_curr = std::get<3>(localEdgeMap[i]);
               std::vector<cv::KeyPoint> keypoints_prev = std::get<0>(globalEdgeMap[j]);
               cv::Mat img_prev = std::get<3>(globalEdgeMap[j]);
               cv::Mat resultMat;
               drawMatches(img_prev, keypoints_prev, img_curr, keypoints_curr, good, resultMat, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
               char windowname[250];
               sprintf(windowname, "Edge curr[%d]-prev[%d]", i, j);
               cv::imshow(windowname, resultMat);
               cv::waitKey(1);
            }
         }

         pcl::PointCloud<pcl::PointXYZI> curr_frame_pc;
         pcl::PointCloud<pcl::PointXYZI> matched_frame_pc;

         int sliding_edge_cnt = 0;
         std::vector<pcl::PointCloud<pcl::PointXYZI>> edge_pair;
         std::vector<std::vector<int>> map_idx;
         int matchedFeatureSize = (int)matchedEdgeFeature[frame_cnt - 1].size();
         for (int i = 0; i < matchedFeatureSize; i++)
         {
            if (matchedEdgeFeature[frame_cnt - 1][i] > -1)
            {
               pcl::PointCloud<pcl::PointXYZI> matched_pc;
               std::vector<int> map_idx_tmp;
               int cnt = 1;
               if (debugging)
               {
                  fprintf(stderr, "matched_edge=[\n");
                  fprintf(stderr, "%f %f %f;\n", pc_curr[i].x, pc_curr[i].y, pc_curr[i].z);
               }
               matched_pc.push_back(pc_curr[i]);
               map_idx_tmp.push_back(frame_cnt - 1);
               print_matchedEdgeFeature(frame_cnt - 1 - 1, matchedEdgeFeature[frame_cnt - 1][i], &cnt, &matched_pc, &map_idx_tmp);
               if (debugging)
                  fprintf(stderr, "];\n");
               if (cnt > 0)
               {
                  sliding_edge_cnt++;
                  edge_pair.push_back(matched_pc);
                  map_idx.push_back(map_idx_tmp);
               
               pcl::PointXYZI temp_pc;
               int matchedPCSize = (int)matched_pc.size();
               // fprintf(stderr, "matchedPCSize: %d\n", matchedPCSize);
               for (int k = 0; k < matchedPCSize; k++)
               {
                  if (k == 0)
                     pc_out_matched->push_back(matched_pc[k]);
                  else
                  {
                     temp_pc.x += matched_pc[k].x;
                     temp_pc.y += matched_pc[k].y;
                     temp_pc.z += matched_pc[k].z;
                     temp_pc.intensity = matched_pc[k].intensity;
                  }
               }
               temp_pc.x /= (matchedPCSize - 1);
               temp_pc.y /= (matchedPCSize - 1);
               temp_pc.z /= (matchedPCSize - 1);
               pc_out_map->push_back(temp_pc);
               }
            }
         }
         // fprintf(stderr, "matched_frame_pc: %d\n", matched_frame_pc.size());
         // for (int k = 0; k < (int)matched_frame_pc.size(); k++)
         // {
         //    fprintf(stderr, "curr_%d:[%f %f %f]\n", k, curr_frame_pc[k].x, curr_frame_pc[k].y, curr_frame_pc[k].z);
         //    fprintf(stderr, "matc_%d:[%f %f %f]\n", k, matched_frame_pc[k].x, matched_frame_pc[k].y, matched_frame_pc[k].z);
         // }

         fprintf(stderr, "----------sliding edge--[%d][%d]----------\n", frame_cnt, sliding_edge_cnt);
         // fprintf(stderr, "start sliding edge test ----[%d]---- \n", edge_pair.size());
         max_dist = max_dist * 2;
         double dist_th = 0;
         int edgePairSize = (int)edge_pair.size();
         for (int i = 0; i < edgePairSize; i++)
         {
            // fprintf(stderr, "matched_edge=[\n");
            if (edge_pair[i].size() > 2)
            {
               Eigen::Vector3d dt;
               dt << edge_pair[i][1].x - edge_pair[i][2].x, edge_pair[i][1].y - edge_pair[i][2].y, edge_pair[i][1].z - edge_pair[i][2].z;
               // for (int j = 0; j < edge_pair[i].size(); j++)
               // {
               //   fprintf(stderr, "%f %f %f;\n", edge_pair[i][j].x, edge_pair[i][j].y, edge_pair[i][j].z);
               // }
               // fprintf(stderr, "];\n");
               double dist = sqrt(dt(0) * dt(0) + dt(1) * dt(1) + dt(2) * dt(2));
               // fprintf(stderr, "dist: %f\n", sqrt(dt(0) * dt(0) + dt(1) * dt(1) + dt(2) * dt(2)));
               if (dist < 0.4)
               {
                  Eigen::Vector3d dt1;
                  dt1 << edge_pair[i][0].x - edge_pair[i][2].x, edge_pair[i][0].y - edge_pair[i][2].y, edge_pair[i][0].z - edge_pair[i][2].z;
                  Eigen::Vector3d dt2;
                  dt2 << edge_pair[i][0].x - edge_pair[i][1].x, edge_pair[i][0].y - edge_pair[i][1].y, edge_pair[i][0].z - edge_pair[i][1].z;
                  double dist1 = sqrt(dt1(0) * dt1(0) + dt1(1) * dt1(1) + dt1(2) * dt1(2));
                  double dist2 = sqrt(dt2(0) * dt2(0) + dt2(1) * dt2(1) + dt2(2) * dt2(2));
                  if (dist1 < dist2)
                  {
                     if (dist_th < dist2)
                        dist_th = dist2;
                  }
                  else
                  {
                     if (dist_th < dist1)
                        dist_th = dist1;
                  }
               }
            }
         }
         if (dist_th > 1.0)
         {
            max_dist = dist_th;
         }

         fprintf(stderr, "max dist: %f\n", max_dist);
         // fprintf(stderr, "total_edge_num: %d\n", total_edge_num);
         fprintf(stderr, "end sliding edge test -------- \n");
      }

      if (debugging)
      {
         cv::waitKey(0);
         fprintf(stderr, "--------------edge---------------------------\n");
      }
   }
}
void VALGENERATIONCLASS::getSurfDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time)
{
   cv::Mat plotImg;
   if (debugging)
      cv::cvtColor(img, plotImg, CV_GRAY2BGR);
   /* projection point cloud on the image*/
   pcl::PointCloud<pcl::PointXYZI> surfPC;
   std::vector<cv::KeyPoint> surfKey;
   std::vector<cv::KeyPoint> update_surfKey;
   pcl::PointCloud<pcl::PointXYZI> update_surfPC;
   cv::Mat update_surfDescriptor;
   std::vector<int> surfFeatureIdx;
   int pointcloudSize = (int)pointcloud_in->size();
   for (int j = 0; j < pointcloudSize; j++)
   {
      pcl::PointXYZI point_temp;
      point_temp.x = pointcloud_in->points[j].x;
      point_temp.y = pointcloud_in->points[j].y;
      point_temp.z = pointcloud_in->points[j].z;
      point_temp.intensity = pointcloud_in->points[j].intensity;
      if (pointcloud_in->points[j].x < 3.0)
      {
         continue;
      }
      Eigen::Vector4d feature_point(point_temp.x, point_temp.y, point_temp.z, 1);
      Eigen::Vector3d homogeneous;
      homogeneous = R0_rect * Tr_velo_to_cam * feature_point;
      Eigen::Vector3d proj_vec = P0 * Eigen::Vector4d(homogeneous(0), homogeneous(1), homogeneous(2), 1);
      proj_vec /= proj_vec(2);

      cv::KeyPoint keyTemp;
      keyTemp.pt = cv::Point2d(lrint(proj_vec(0)), lrint(proj_vec(1)));
      if (keyTemp.pt.x > 0 && keyTemp.pt.x < img.cols && keyTemp.pt.y > 0 && keyTemp.pt.y < img.rows)
      {
         surfKey.push_back(keyTemp);
         update_surfKey.push_back(keyTemp);
         surfPC.push_back(point_temp);
      }
   }
   /***** update key point to descriptor*******/
   const static auto &orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2);
   orb->compute(img, update_surfKey, update_surfDescriptor);
   int updateSurfKeySize = (int)update_surfKey.size();
   for (int j = 0; j < updateSurfKeySize; j++)
   {
      cv::Point2d update_key = update_surfKey[j].pt;
      if (debugging)
      {
         cv::circle(plotImg, update_key, 2, cv::Scalar(0, 0, 255), -1);
      }
      int surfKeySize = (int)surfKey.size();
      for (int i = 0; i < surfKeySize; i++)
      {
         cv::Point2d key_tmp = surfKey[i].pt;

         double dist_x = update_key.x - key_tmp.x;
         double dist_y = update_key.y - key_tmp.y;
         double dist = dist_x * dist_x + dist_y * dist_y;
         if (dist < 1)
         {
            update_surfPC.push_back(surfPC[i]);
            break;
         }
      }
   }
   for (int i = 0; i < updateSurfKeySize; i++)
   {
      surfFeatureIdx.push_back(-1);
   }
   matchedSurfFeature.push_back(surfFeatureIdx);
   if (updateSurfKeySize > 0 && update_surfDescriptor.rows > 1)
   {
      if (debugging)
      {
         localSurfMap.push_back(make_tuple(update_surfKey, update_surfDescriptor, update_surfPC, plotImg, odom, sub_time));
         cv::imshow("surf curr img", plotImg);
         cv::waitKey(1);
      }
      else
      {
         cv::Mat temp;
         localSurfMap.push_back(make_tuple(update_surfKey, update_surfDescriptor, update_surfPC, temp, odom, sub_time));
      }
   }
}
void VALGENERATIONCLASS::getEdgeDescriptor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time)
{
   cv::Mat plotImg;
   if (debugging)
      cv::cvtColor(img, plotImg, CV_GRAY2BGR);
   /* projection point cloud on the image*/
   pcl::PointCloud<pcl::PointXYZI> edgePC;
   std::vector<cv::KeyPoint> edgeKey;
   std::vector<cv::KeyPoint> update_edgeKey;
   pcl::PointCloud<pcl::PointXYZI> update_edgePC;
   cv::Mat update_edgeDescriptor;
   std::vector<int> edgeFeatureIdx;
   int pointcloudSize = (int)pointcloud_in->size();
   for (int j = 0; j < pointcloudSize; j++)
   {
      pcl::PointXYZI point_temp;
      point_temp.x = pointcloud_in->points[j].x;
      point_temp.y = pointcloud_in->points[j].y;
      point_temp.z = pointcloud_in->points[j].z;
      point_temp.intensity = pointcloud_in->points[j].intensity;
      if (pointcloud_in->points[j].x < 3.0)
      {
         continue;
      }
      Eigen::Vector4d feature_point(point_temp.x, point_temp.y, point_temp.z, 1);
      Eigen::Vector3d homogeneous;
      homogeneous = R0_rect * Tr_velo_to_cam * feature_point;
      Eigen::Vector3d proj_vec = P0 * Eigen::Vector4d(homogeneous(0), homogeneous(1), homogeneous(2), 1);
      proj_vec /= proj_vec(2);

      cv::KeyPoint keyTemp;
      keyTemp.pt = cv::Point2d(lrint(proj_vec(0)), lrint(proj_vec(1)));
      if (keyTemp.pt.x > 0 && keyTemp.pt.x < img.cols && keyTemp.pt.y > 0 && keyTemp.pt.y < img.rows)
      {
         edgeKey.push_back(keyTemp);
         update_edgeKey.push_back(keyTemp);
         edgePC.push_back(point_temp);
      }
   }
   /***** update key point to descriptor*******/
   // const static auto &orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2);
   // orb->compute(img, update_edgeKey, update_edgeDescriptor);
   // fprintf(stderr, "update_edgeKey size: %d\n", update_edgeKey.size());
   // cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
   // akaze->compute(img ,update_edgeKey, update_edgeDescriptor);
   // fprintf(stderr, "end compute akaze!!!!\n");
   const static auto &brisk = cv::BRISK::create();
   brisk->compute(img, update_edgeKey, update_edgeDescriptor);
   

   int updateEdgeKeySize = (int)update_edgeKey.size();
   for (int j = 0; j < updateEdgeKeySize; j++)
   {
      cv::Point2d update_key = update_edgeKey[j].pt;
      if (debugging)
      {
         cv::circle(plotImg, update_key, 2, cv::Scalar(0, 0, 255), -1);
      }
      int edgeKeySize = (int)edgeKey.size();
      for (int i = 0; i < edgeKeySize; i++)
      {
         cv::Point2d key_tmp = edgeKey[i].pt;
         double dist_x = update_key.x - key_tmp.x;
         double dist_y = update_key.y - key_tmp.y;
         double dist = dist_x * dist_x + dist_y * dist_y;
         if (dist < 1)
         {
            update_edgePC.push_back(edgePC[i]);
            break;
         }
      }
   }
   for (int i = 0; i < updateEdgeKeySize; i++)
   {
      edgeFeatureIdx.push_back(-1);
   }
   matchedEdgeFeature.push_back(edgeFeatureIdx);
   fprintf(stderr, "update edge key size: %d\n", updateEdgeKeySize);
   if (updateEdgeKeySize > 0 && update_edgeDescriptor.rows > 1)
   {
      if (debugging)
      {
         localEdgeMap.push_back(make_tuple(update_edgeKey, update_edgeDescriptor, update_edgePC, plotImg, odom, sub_time));
         cv::imshow("curr img", plotImg);
         cv::waitKey(1);
      }
      else
      {
         cv::Mat temp;
         localEdgeMap.push_back(make_tuple(update_edgeKey, update_edgeDescriptor, update_edgePC, temp, odom, sub_time));
      }
   }
}
void VALGENERATIONCLASS::planeDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointcloud_in, cv::Mat img, ros::Time sub_time)
{
   int regions[100];
   const int region_max = 30;

   cv::Mat plotImg;
   if (debugging)
      cv::cvtColor(img, plotImg, CV_GRAY2BGR);

   /*** Downsampling + ground & ceiling removal ***/
   pcl::IndicesPtr pc_indices(new std::vector<int>);
   int pointcloudSize = (int)pointcloud_in->size();
   for (int i = 0; i < pointcloudSize; ++i)
   {
      if (i % leaf == 0)
      {
         if (pointcloud_in->points[i].z >= object_z_axis_min && pointcloud_in->points[i].z <= object_z_axis_max && pointcloud_in->points[i].x > object_x_axis_min && pointcloud_in->points[i].x < object_x_axis_max && pointcloud_in->points[i].y > object_y_axis_min && pointcloud_in->points[i].y < object_y_axis_max)
         {
            pc_indices->push_back(i);
         }
      }
   }

   /*** Divide the point cloud into nested circular regions ***/
   boost::array<std::vector<int>, region_max> indices_array;
   int IndicesSize = (int)pc_indices->size();
   for (int i = 0; i < IndicesSize; i++)
   {
      float range = 0.0;
      for (int j = 0; j < region_max; j++)
      {
         float d2 = pointcloud_in->points[(*pc_indices)[i]].x * pointcloud_in->points[(*pc_indices)[i]].x +
                    pointcloud_in->points[(*pc_indices)[i]].y * pointcloud_in->points[(*pc_indices)[i]].y +
                    pointcloud_in->points[(*pc_indices)[i]].z * pointcloud_in->points[(*pc_indices)[i]].z;
         if (d2 > object_radius_min * object_radius_min && d2 < object_radius_max * object_radius_max &&
             d2 > range * range && d2 <= (range + regions[j]) * (range + regions[j]))
         {
            indices_array[j].push_back((*pc_indices)[i]);
            break;
         }
         range += regions[j];
      }
   }

   /*** Euclidean clustering ***/
   float tolerance = 0.0;
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
   int last_clusters_begin = 0;
   int last_clusters_end = 0;

   for (int i = 0; i < region_max; i++)
   {
      tolerance += 0.1;
      if (indices_array[i].size() > cluster_size_min)
      {
         boost::shared_ptr<std::vector<int>> indices_array_ptr(new std::vector<int>(indices_array[i]));
         pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
         tree->setInputCloud(pointcloud_in, indices_array_ptr);

         std::vector<pcl::PointIndices> cluster_indices;
         pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
         ec.setClusterTolerance(tolerance);
         ec.setMinClusterSize(cluster_size_min);
         // ec.setMaxClusterSize(cluster_size_max_);
         ec.setSearchMethod(tree);
         ec.setInputCloud(pointcloud_in);
         ec.setIndices(indices_array_ptr);
         ec.extract(cluster_indices);

         for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
         {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
               cluster->points.push_back(pointcloud_in->points[*pit]);
            }
            /*** Merge clusters separated by nested regions ***/
            for (int j = last_clusters_begin; j < last_clusters_end; j++)
            {
               pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
               int K = 1; // the number of neighbors to search for
               std::vector<int> k_indices(K);
               std::vector<float> k_sqr_distances(K);
               kdtree.setInputCloud(cluster);
               if (clusters[j]->points.size() >= 1)
               {
                  if (kdtree.nearestKSearch(*clusters[j], clusters[j]->points.size() - 1, K, k_indices, k_sqr_distances) > 0)
                  {
                     if (k_sqr_distances[0] < object_k_merging_threshold)
                     {
                        *cluster += *clusters[j];
                        clusters.erase(clusters.begin() + j);
                        last_clusters_end--;
                        // std::cerr << "k-merging: clusters " << j << " is merged" << std::endl;
                     }
                  }
               }
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
         }

         last_clusters_begin = last_clusters_end;
         last_clusters_end = clusters.size();
      }
   }
   /*********** Projection of ground point cloud onto image ***************/
   /***************** detect object plane *********************/
   int clusterSize = (int)clusters.size();
   for (int k = 0; k < clusterSize; k++)
   {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      cluster = clusters[k];

      /*detect detect plane */
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.1);
      int original_size(cluster->height * cluster->width);
      float _min_percentage = 70.0;

      while (cluster->height * cluster->width > original_size * _min_percentage / 100.0)
      {
         cv::Mat plane_descriptor(1, 1, CV_8UC1);
         std::vector<cv::KeyPoint> plane_keypoints;
         pcl::PointCloud<pcl::PointXYZI> plane_pointclouds;
         seg.setInputCloud(cluster);

         seg.segment(*inliers, *coefficients);
         if (inliers->indices.size() < cluster_size_min || coefficients->values[2] > 0.5)
         {
            // std::cerr << "Could not estimate a planar model for the given cluster points" << std::endl;
            break;
         }

         /***************************extract plane point cloud******************************/
         pcl::ExtractIndices<pcl::PointXYZI> extract;
         extract.setInputCloud(cluster);
         extract.setIndices(inliers);
         extract.setNegative(false);
         pcl::PointCloud<pcl::PointXYZI> cloudF;
         extract.filter(cloudF);

         /***************************get image feature from plane******************************/
         /* projection point cloud on the image*/
         bool front_plane = true;
         pcl::PointCloud<pcl::PointXYZI> planePC;
         std::vector<cv::KeyPoint> planeKey;
         std::vector<cv::KeyPoint> update_planeKey;
         pcl::PointCloud<pcl::PointXYZI> update_planePC;
         cv::Mat update_planeDescriptor;
         int cloudFSize = (int)cloudF.size();
         for (int j = 0; j < cloudFSize; j++)
         {
            pcl::PointXYZI point_temp;
            point_temp.x = cloudF[j].x;
            point_temp.y = cloudF[j].y;
            point_temp.z = cloudF[j].z;
            point_temp.intensity = cloudF[j].intensity;
            if (cloudF[j].x < 1.0)
            {
               front_plane = false;
               break;
            }
            Eigen::Vector4d feature_point(point_temp.x, point_temp.y, point_temp.z, 1);
            Eigen::Vector3d homogeneous;
            homogeneous = R0_rect * Tr_velo_to_cam * feature_point;
            Eigen::Vector3d proj_vec = P0 * Eigen::Vector4d(homogeneous(0), homogeneous(1), homogeneous(2), 1);
            proj_vec /= proj_vec(2);

            cv::KeyPoint keyTemp;
            keyTemp.pt = cv::Point2d(lrint(proj_vec(0)), lrint(proj_vec(1)));
            if (keyTemp.pt.x > 0 && keyTemp.pt.x < img.cols && keyTemp.pt.y > 0 && keyTemp.pt.y < img.rows)
            {
               planeKey.push_back(keyTemp);
               update_planeKey.push_back(keyTemp);
               planePC.push_back(cloudF[j]);
            }
         }

         /* calculate plane box in the image frame*/
         if (front_plane)
         {
            /***** update key point to descriptor*******/
            const static auto &orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2);
            orb->compute(img, update_planeKey, update_planeDescriptor);
            int updatePlaneKeySize = update_planeKey.size();
            for (int j = 0; j < updatePlaneKeySize; j++)
            {
               cv::Point2d update_key = update_planeKey[j].pt;
               int planeKeySize = planeKey.size();
               for (int i = 0; i < planeKeySize; i++)
               {
                  cv::Point2d key_tmp = planeKey[i].pt;
                  double dist_x = update_key.x - key_tmp.x;
                  double dist_y = update_key.y - key_tmp.y;
                  double dist = dist_x * dist_x + dist_y * dist_y;
                  if (dist < 1)
                  {
                     update_planePC.push_back(planePC[i]);
                     break;
                  }
               }
            }

            /***save plane local map***/
            if (update_planeKey.size() > 0 && update_planeDescriptor.rows > 1)
            {
               if (debugging)
               {
                  localPlaneMap.push_back(make_tuple(update_planeKey, update_planeDescriptor, update_planePC, plotImg, odom, sub_time));
               }
               else
               {
                  cv::Mat temp;
                  localPlaneMap.push_back(make_tuple(update_planeKey, update_planeDescriptor, update_planePC, temp, odom, sub_time));
               }
            }
         }
         if (front_plane && debugging)
         {
            cv::Point2d minPoint;
            cv::Point2d maxPoint;

            cv::Point2d leftTop;
            cv::Point2d rightTop;
            cv::Point2d leftBot;
            cv::Point2d rightBot;
            int planeKeySize = planeKey.size();
            for (int j = 0; j < planeKeySize; j++)
            {
               cv::Point2d tmp_pt = planeKey[j].pt;
               if (j == 0)
               {
                  minPoint = tmp_pt;
                  maxPoint = tmp_pt;
               }
               else
               {
                  if (minPoint.x > tmp_pt.x)
                     minPoint.x = tmp_pt.x;
                  if (maxPoint.x < tmp_pt.x)
                     maxPoint.x = tmp_pt.x;
                  if (minPoint.y > tmp_pt.y)
                     minPoint.y = tmp_pt.y;
                  if (maxPoint.y < tmp_pt.y)
                     maxPoint.y = tmp_pt.y;
               }
            }

            minPoint.x = MAX(minPoint.x, 0);
            minPoint.y = MAX(minPoint.y, 0);
            minPoint.x = MIN(minPoint.x, img.cols);
            minPoint.y = MIN(minPoint.y, img.rows);

            maxPoint.x = MAX(maxPoint.x, 0);
            maxPoint.y = MAX(maxPoint.y, 0);
            maxPoint.y = MIN(maxPoint.y, img.rows);
            maxPoint.x = MIN(maxPoint.x, img.cols);

            leftTop = minPoint;
            rightTop.x = maxPoint.x;
            rightTop.y = minPoint.y;
            leftBot.x = minPoint.x;
            leftBot.y = maxPoint.y;
            rightBot = maxPoint;

            if (leftTop.x != rightTop.x && leftBot.x != rightBot.x)
            {

               cv::line(plotImg, leftTop, rightTop, cv::Scalar(0, 255, 0), 2);
               cv::line(plotImg, leftTop, leftBot, cv::Scalar(0, 255, 0), 2);
               cv::line(plotImg, leftBot, rightBot, cv::Scalar(0, 255, 0), 2);
               cv::line(plotImg, rightTop, rightBot, cv::Scalar(0, 255, 0), 2);
            }
            for (int j = 0; j < planeKeySize; j++)
               cv::circle(plotImg, planeKey[j].pt, 2, cv::Scalar(0, 0, 255), -1);
            int updatePlaneKeySize = update_planeKey.size();
            for (int i = 0; i < updatePlaneKeySize; i++)
               cv::circle(plotImg, update_planeKey[i].pt, 2, cv::Scalar(255, 0, 255), -1);
         }

         /************repeat plane detection ***************************/
         extract.setNegative(true);
         pcl::PointCloud<pcl::PointXYZI> outliers;
         extract.filter(outliers);
         cluster->swap(outliers);

         /***************************************************************/
      }
   }
}

void VALGENERATIONCLASS::updateMap(ros::Time odom_time, Eigen::Isometry3d odom_in)
{
   fprintf(stderr, "update map!!!!\n");
   // update Edge map
   int mapIndex = -1;
   int sizeMap = (int)localEdgeMap.size();
   fprintf(stderr, "localMapSize: %d\n", sizeMap);
   for (int i = sizeMap - 1; i >= 0; i--)
   {
      double pointcloudTime1 = std::get<5>(localEdgeMap[i]).toSec();
      if (abs(pointcloudTime1 - odom_time.toSec()) < 0.05)
      {
         mapIndex = i;
         std::get<4>(localEdgeMap[i]) = odom_in;

         if (mapIndex > -1)
         {
            // update edge Map
            pcl::PointCloud<pcl::PointXYZI> localPointCloud = std::get<2>(localEdgeMap[mapIndex]);
            pcl::PointCloud<pcl::PointXYZI> globalPointCloud;
            Eigen::Isometry3d globalPoistion = std::get<4>(localEdgeMap[mapIndex]);
            Eigen::Quaterniond q_current(globalPoistion.rotation());
            Eigen::Vector3d t_current = globalPoistion.translation();
            int pointcloudSize = (int)localPointCloud.size();
            for (int j = 0; j < pointcloudSize; j++)
            {
               Eigen::Vector3d point_curr(localPointCloud[j].x, localPointCloud[j].y, localPointCloud[j].z);
               Eigen::Vector3d point_w = q_current * point_curr + t_current;
               pcl::PointXYZI po;
               po.x = point_w.x();
               po.y = point_w.y();
               po.z = point_w.z();
               po.intensity = localPointCloud[j].intensity;
               globalPointCloud.push_back(po);
            }
            globalEdgeMap.push_back(make_tuple(std::get<0>(localEdgeMap[mapIndex]), std::get<1>(localEdgeMap[mapIndex]), globalPointCloud, std::get<3>(localEdgeMap[mapIndex]), std::get<4>(localEdgeMap[mapIndex]), std::get<5>(localEdgeMap[mapIndex])));
            matchedEdgePC.push_back(globalPointCloud);
            localEdgeMap.clear();
         }
      }
   }
   if (globalEdgeMap.size() > 15)
      for (int i = 0; i < 10; i++)
         globalEdgeMap.erase(globalEdgeMap.begin());

   // update surf map
   sizeMap = (int)localSurfMap.size();
   fprintf(stderr, "localMapSize: %d\n", sizeMap);
   for (int i = sizeMap - 1; i >= 0; i--)
   {
      double pointcloudTime1 = std::get<5>(localSurfMap[i]).toSec();
      if (abs(pointcloudTime1 - odom_time.toSec()) < 0.05)
      {
         mapIndex = i;
         std::get<4>(localSurfMap[i]) = odom_in;

         if (mapIndex > -1)
         {
            // update edge Map
            pcl::PointCloud<pcl::PointXYZI> localPointCloud = std::get<2>(localSurfMap[mapIndex]);
            pcl::PointCloud<pcl::PointXYZI> globalPointCloud;
            Eigen::Isometry3d globalPoistion = std::get<4>(localSurfMap[mapIndex]);
            Eigen::Quaterniond q_current(globalPoistion.rotation());
            Eigen::Vector3d t_current = globalPoistion.translation();
            int pointcloudSize = (int)localPointCloud.size();
            for (int j = 0; j < pointcloudSize; j++)
            {
               Eigen::Vector3d point_curr(localPointCloud[j].x, localPointCloud[j].y, localPointCloud[j].z);
               Eigen::Vector3d point_w = q_current * point_curr + t_current;
               pcl::PointXYZI po;
               po.x = point_w.x();
               po.y = point_w.y();
               po.z = point_w.z();
               po.intensity = localPointCloud[j].intensity;
               globalPointCloud.push_back(po);
            }
            globalSurfMap.push_back(make_tuple(std::get<0>(localSurfMap[mapIndex]), std::get<1>(localSurfMap[mapIndex]), globalPointCloud, std::get<3>(localSurfMap[mapIndex]), std::get<4>(localSurfMap[mapIndex]), std::get<5>(localSurfMap[mapIndex])));
            matchedSurfPC.push_back(globalPointCloud);
            localSurfMap.clear();
         }
      }
   }
   if (globalSurfMap.size() > 15)
      for (int i = 0; i < 10; i++)
         globalSurfMap.erase(globalSurfMap.begin());
}