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
 * multi_lidar_fusion.cpp
 *
 *  Created on: May 3, 2018
 */

#include "multi_lidar_fusion.h"


void RosMultiLidarFusionApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = parent_frame_;
  in_publisher.publish(cloud_msg);
}

void RosMultiLidarFusionApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                            const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud1_msg,
                                            const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud2_msg)
{
  tf::TransformListener listener_child1, listener_child2;
  tf::StampedTransform transform_child1, transform_child2;  
  
  pcl::PointCloud<PointT>::Ptr in_parent_cloud_ptr(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_child_cloud1_ptr(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr in_child_cloud2_ptr(new pcl::PointCloud<PointT>);    
   
  pcl::PointCloud<PointT> output_cloud;  

  pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud_ptr);
  pcl::fromROSMsg(*in_child_cloud1_msg, *in_child_cloud1_ptr);
  pcl::fromROSMsg(*in_child_cloud2_msg, *in_child_cloud2_ptr);
  
  output_cloud+=*in_parent_cloud_ptr; 

  parent_frame_ = in_parent_cloud_msg->header.frame_id;
  child_frame1_ = in_child_cloud1_msg->header.frame_id;
  child_frame2_ = in_child_cloud2_msg->header.frame_id;
  
  // Transforming unfiltered, input cloud using found transform.
  /*
  try
  {
    ros::Time now = ros::Time(0);
    listener_child1.waitForTransform(child_frame1_, parent_frame_, now, ros::Duration(10.0));
    listener_child1.lookupTransform(child_frame1_, parent_frame_, now, transform_child1);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }  
  pcl_ros::transformPointCloud (*in_child_cloud1_ptr, *in_child_cloud1_ptr, transform_child1.inverse());  
  */
  pcl::transformPointCloud(*in_child_cloud1_ptr, *in_child_cloud1_ptr, tf_fltol_);
  output_cloud+=*in_child_cloud1_ptr; 
  /*
  try
  {
    ros::Time now = ros::Time(0);
    listener_child2.waitForTransform(child_frame2_, parent_frame_, now, ros::Duration(10.0));
    listener_child2.lookupTransform(child_frame2_, parent_frame_, now, transform_child2);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  pcl_ros::transformPointCloud (*in_child_cloud2_ptr, *in_child_cloud2_ptr, transform_child2.inverse());  
  */
  pcl::transformPointCloud(*in_child_cloud2_ptr, *in_child_cloud2_ptr, tf_frtol_);
  output_cloud+=*in_child_cloud2_ptr; 
  
  pcl::PointCloud<PointT>::Ptr output_cloud_ptr(new pcl::PointCloud<PointT>(output_cloud));  

  PublishCloud(fused_cloud_publisher_, output_cloud_ptr);
  // timer end
  //auto end = std::chrono::system_clock::now();
  //auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  //std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;
}

void RosMultiLidarFusionApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string points_parent_topic_str, points_child_topic_str1, points_child_topic_str2;
  
  std::string fused_points_topic_str = "/points_raw";
  
  if (in_private_handle.getParam("localizer_fl", localizer_fl_) == false)
  {
    std::cout << "localizer_fl is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_x", fl_tf_x_) == false)
  {
    std::cout << "fl_tf_x is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_y", fl_tf_y_) == false)
  {
    std::cout << "fl_tf_y is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_z", fl_tf_z_) == false)
  {
    std::cout << "fl_tf_z is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_roll", fl_tf_roll_) == false)
  {
    std::cout << "fl_tf_roll is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_pitch", fl_tf_pitch_) == false)
  {
    std::cout << "fl_tf_pitch is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fl_tf_yaw", fl_tf_yaw_) == false)
  {
    std::cout << "fl_tf_yaw is not set." << std::endl;
    exit(1);
  }
  
  ROS_INFO("[%s] (fl_tf_x_,fl_tf_y_,fl_tf_z_,fl_tf_roll_,fl_tf_pitch_,fl_tf_yaw_): (%f %f %f %f %f %f)",
           __APP_NAME__, fl_tf_x_,fl_tf_y_,fl_tf_z_,fl_tf_roll_,fl_tf_pitch_,fl_tf_yaw_);
  
  if (in_private_handle.getParam("localizer_fr", localizer_fr_) == false)
  {
    std::cout << "localizer_fr is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_x", fr_tf_x_) == false)
  {
    std::cout << "fr_tf_x is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_y", fr_tf_y_) == false)
  {
    std::cout << "fr_tf_y is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_z", fr_tf_z_) == false)
  {
    std::cout << "fr_tf_z is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_roll", fr_tf_roll_) == false)
  {
    std::cout << "fr_tf_roll is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_pitch", fr_tf_pitch_) == false)
  {
    std::cout << "fr_tf_pitch is not set." << std::endl;
    exit(1);
  }
  if (in_private_handle.getParam("fr_tf_yaw", fr_tf_yaw_) == false)
  {
    std::cout << "fr_tf_yaw is not set." << std::endl;
    exit(1);
  }
  
  ROS_INFO("[%s] (fr_tf_x_,fr_tf_y_,fr_tf_z_,fr_tf_roll_,fr_tf_pitch_,fr_tf_yaw_): (%f %f %f %f %f %f)",
           __APP_NAME__, fr_tf_x_,fr_tf_y_,fr_tf_z_,fr_tf_roll_,fr_tf_pitch_,fr_tf_yaw_);
  
  Eigen::Translation3f tl_fltol(fl_tf_x_, fl_tf_y_, fl_tf_z_);                 // tl: translation
  Eigen::AngleAxisf rot_x_fltol(fl_tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_fltol(fl_tf_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_fltol(fl_tf_yaw_, Eigen::Vector3f::UnitZ());
  tf_fltol_ = (tl_fltol * rot_z_fltol * rot_y_fltol * rot_x_fltol).matrix();
  
  Eigen::Translation3f tl_frtol(fr_tf_x_, fr_tf_y_, fr_tf_z_);                 // tl: translation
  Eigen::AngleAxisf rot_x_frtol(fr_tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_frtol(fr_tf_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_frtol(fr_tf_yaw_, Eigen::Vector3f::UnitZ());
  tf_frtol_ = (tl_frtol * rot_z_frtol * rot_y_frtol * rot_x_frtol).matrix();
  

  in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
  ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

  in_private_handle.param<std::string>("points_child_src1", points_child_topic_str1, "points_raw");
  ROS_INFO("[%s] points_child_src1: %s",__APP_NAME__, points_child_topic_str1.c_str());
  
  in_private_handle.param<std::string>("points_child_src2", points_child_topic_str2, "points_raw");
  ROS_INFO("[%s] points_child_src2: %s",__APP_NAME__, points_child_topic_str2.c_str());

  //generate subscribers and synchronizer
  cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                       points_parent_topic_str, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

  cloud_child_subscriber1_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                       points_child_topic_str1, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str1.c_str());
  
  cloud_child_subscriber2_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                                       points_child_topic_str2, 10);
  ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str2.c_str());  

  fused_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(fused_points_topic_str, 1);
  ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, fused_points_topic_str.c_str());

  cloud_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
                                                     *cloud_parent_subscriber_,
                                                     *cloud_child_subscriber1_,
                                                     *cloud_child_subscriber2_);
  cloud_synchronizer_->registerCallback(boost::bind(&RosMultiLidarFusionApp::PointsCallback, this, _1, _2, _3));

}


void RosMultiLidarFusionApp::Run()
{
  ros::NodeHandle private_node_handle("~");

  InitializeRosIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END",__APP_NAME__);
}

RosMultiLidarFusionApp::RosMultiLidarFusionApp()
{
  
}