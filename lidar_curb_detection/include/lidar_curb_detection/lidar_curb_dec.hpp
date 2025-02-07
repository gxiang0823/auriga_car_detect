/*
 * @Description: 
 * @Author: speedzjy
 * @Date: 2022-04-27 11:49:31
 */

#pragma once

#ifndef LIDAR_CURB_DEC_H
#define LIDAR_CURB_DEC_H

#include "lidar_curb_detection/curb_utils.hpp"
#include "lidar_curb_detection/cloud_mapper.hpp"
#include "lidar_curb_detection/ground_segment.hpp"
#include "lidar_curb_detection/feature_points.hpp"
#include "lidar_curb_detection/boundary_points.hpp"


namespace CurbDectection {

class LidarCurbDectection : public ParamServer {

public:
  LidarCurbDectection();
  ~LidarCurbDectection();

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
  void transformCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg);

  // 输入的每帧点云
  PointCloudType::Ptr complete_points;
  CloudQueue queue_complete_points;
  // 边界点云
  CloudPtrList boundary_points;

private:
  ros::Subscriber subPointCloud_;
  ros::Subscriber subTFstamped_;

  ros::Publisher pubCompleteCloud_;
  ros::Publisher pubGroundCloud_;
  ros::Publisher pubNoGroundCloud_;
  ros::Publisher pubFeatureCloud_;
  ros::Publisher pubCurbCloudLeft_;
  ros::Publisher pubCurbCloudRight_;
  ros::Publisher pubMarker_;
  //曲线拟合结果发布
  ros::Publisher curb_fitting_;
  ros::Publisher marker_pub_;
  
};

}

#endif