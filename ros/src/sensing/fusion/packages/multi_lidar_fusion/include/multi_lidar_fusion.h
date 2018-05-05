/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: izeki (izeki.tw@gmail.com)
 *
 * multi_lidar_fusion.h
 *
 *  Created on: May 3, 2018
 */

#ifndef MULTI_LIDAR_FUSION_H
#define MULTI_LIDAR_FUSION_H


#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <tf/tf.h>

#define __APP_NAME__ "multi_lidar_fusion"

class RosMultiLidarFusionApp

{
  ros::NodeHandle                     node_handle_;
  ros::Publisher                      fused_cloud_publisher_;  

  std::string                         parent_frame_;
  std::string                         child_frame1_;
  std::string                         child_frame2_;

  typedef
  message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,  
                                                  sensor_msgs::PointCloud2,  
                                                  sensor_msgs::PointCloud2>   SyncPolicyT;

  typedef pcl::PointXYZI              PointT;

  message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_parent_subscriber_, 
                                                          *cloud_child_subscriber1_,
                                                          *cloud_child_subscriber2_;
  
  message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

  /*!
   * Receives 3 synchronized point cloud messages.
   * @param[in] in_parent_cloud_msg Message containing pointcloud classified as ground.
   * @param[in] in_child_cloud1_msg Message containing pointcloud classified as obstacle.
   * @param[in] in_child_cloud2_msg Message containing pointcloud classified as obstacle.
   */
  void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_parent_cloud_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& in_child_cloud1_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& in_child_cloud2_msg);

  //void InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose);

  /*!
   * Obtains parameters from the command line, initializes subscribers and publishers.
   * @param in_private_handle Ros private handle to get parameters for this node.
   */
  void InitializeRosIo(ros::NodeHandle& in_private_handle);


  /*!
   * Publishes a PointCloud in the specified publisher
   * @param in_publisher Publisher to use
   * @param in_cloud_to_publish_ptr Cloud to Publish
   */
  void PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr);

public:
  void Run();

  RosMultiLidarFusionApp();
};

#endif //MULTI_LIDAR_FUSION_H
