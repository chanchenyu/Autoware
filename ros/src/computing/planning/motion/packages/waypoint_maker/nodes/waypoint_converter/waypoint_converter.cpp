/*
 *  Copyright (c) 2015, Nagoya University
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
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>

#include <iostream>
#include <vector>
#include <string>

#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/waypoint.h"
#include "path_follower_msgs/TrajectoryPoint2D.h"
#include "path_follower_msgs/Trajectory2D.h"

namespace
{
ros::Publisher final_pub;

const double RADIUS_MAX_ = 9e10;
const double KAPPA_MIN_ = 1/RADIUS_MAX_;
  
  
double calcCurvature(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) 
{
  double kappa;
  double denominator = pow(getPlaneDistance(p2.pose.position, p1.pose.position), 2);
  double numerator = 2 * calcRelativeCoordinate(p2.pose.position, p1.pose).y;

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if (numerator > 0)
      kappa = KAPPA_MIN_;
    else
      kappa = -KAPPA_MIN_;
  }
  //ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

void finalCallback(const autoware_msgs::lane &final_waypoints)
{
  path_follower_msgs::Trajectory2D trajectory_waypoints;
  double kappa=-1, theta=-1;
  for (unsigned int i = 1; i <= final_waypoints.waypoints.size(); i++)
  {
    path_follower_msgs::TrajectoryPoint2D trajectory_waypoint_0;
    autoware_msgs::waypoint waypoint_0 = final_waypoints.waypoints[i-1];
    if (i < final_waypoints.waypoints.size()) {            
      path_follower_msgs::TrajectoryPoint2D trajectory_waypoint_1;
      autoware_msgs::waypoint waypoint_1 = final_waypoints.waypoints[i];
      kappa = calcCurvature(waypoint_0.pose, waypoint_1.pose);
      theta = atan2((waypoint_1.pose.pose.position.y - waypoint_0.pose.pose.position.y),
                    (waypoint_1.pose.pose.position.x - waypoint_0.pose.pose.position.x));
    }  
    trajectory_waypoint_0.x = waypoint_0.pose.pose.position.x;
    trajectory_waypoint_0.y = waypoint_0.pose.pose.position.y;
    trajectory_waypoint_0.theta = theta;
    trajectory_waypoint_0.v = waypoint_0.twist.twist.linear.x;
    trajectory_waypoint_0.kappa = kappa;
    trajectory_waypoints.point.push_back(trajectory_waypoint_0);
  }
  final_pub.publish(trajectory_waypoints);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoints_converter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");  

  //subscribe local waypoints
  ros::Subscriber final_sub = nh.subscribe("final_waypoints", 10, finalCallback);
	
  //publish converted local waypoints trajectory
  final_pub = nh.advertise<path_follower_msgs::Trajectory2D>("final_trajectory", 1);
	
  ros::spin();

}
