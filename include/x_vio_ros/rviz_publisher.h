/*
 * Copyright 2020 California  Institute  of Technology (“Caltech”)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RVIZPUBLISHER_H_
#define RVIZPUBLISHER_H_

#include "x/vio/types.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#define OPP_LIFETIME 1
#define OPP_SCALE 0.02
#define PER_SCALE 0.06

class rvizPublisher {
 public:
  rvizPublisher()
      : n(),
        trail_counter(0) {}

  void setUpPublishers(const int n_poses_max, const int n_slam_features_max) {
    marker_opp_in_ =
      n.advertise<visualization_msgs::Marker>("opportunistic_inliers", n_poses_max);
    marker_opp_out_pub_ =
      n.advertise<visualization_msgs::Marker>("opportunistic_outliers", n_poses_max);
    marker_per_pub_
      = n.advertise<visualization_msgs::Marker>("persistent_features", n_slam_features_max);
  }

  void publishMsckfFeatures(const x::Vector3dArray& inliers,
		  	      const x::Vector3dArray& outliers)
  {
    // Inliers
    for(size_t i=0; i < inliers.size(); i++)
    {
      const Eigen::Vector3d& feature(inliers[i]);
      publish3dFeature(feature[0],
		       feature[1],
		       feature[2],
		       0,
		       false);
    }
    
    // Outliers
    for(size_t j=0; j < outliers.size(); j++)
    {
      const Eigen::Vector3d& feature(outliers[j]);
      publish3dFeature(feature[0],
		       feature[1],
		       feature[2],
		       0,
		       true);
    }
  }

  void publish3dFeature(const double x,
                        const double y,
                        const double z,
                        const double id,
                        const int outlier)
  {
    // Visual marker initialization
    visualization_msgs::Marker point;
    point.header.frame_id = "world";
    point.header.stamp    = ros::Time::now();
    point.ns = "msckf_features";
    point.id              = trail_counter;
    point.type            = visualization_msgs::Marker::CUBE;
    point.action          = visualization_msgs::Marker::ADD;
    point.lifetime        = ros::Duration(OPP_LIFETIME);
    point.scale.x         = OPP_SCALE;
    point.scale.y         = OPP_SCALE;
    point.scale.z         = OPP_SCALE;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = z;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    if (outlier)
    {
      // Opportunistic outliers are black
      point.color.r = 0.0;
      point.color.g = 0.0;
      point.color.b = 0.0;
      point.color.a = 1.0;

      marker_opp_out_pub_.publish(point);
    }
    else
    {
      // Opportunistic inliers are orange
      point.color.r = 200.0 / 255.0;
      point.color.g = 140.0 / 255.0;
      point.color.b = 45.0 / 255.0;
      point.color.a = 1.0f;

      marker_opp_in_.publish(point);
    }

    trail_counter++;
  }

  // Delete all persistent features printed in rviz
  void deleteAllPersistentFeatures()
  {
    visualization_msgs::Marker del_msg;
    del_msg.header.frame_id = "world";
    del_msg.action = 3; // DELETEALL

    marker_per_pub_.publish(del_msg);
  }

  void publishPersistentFeature(double const x,
                                double const y,
                                double const z,
                                double const id)
  {
    visualization_msgs::Marker point;

    point.header.frame_id = "world";
    point.header.stamp = ros::Time::now();
    point.action = visualization_msgs::Marker::ADD;
    point.lifetime = ros::Duration();
    point.ns = "slam_features";
    point.id = id;
    point.type = visualization_msgs::Marker::CUBE;

    point.scale.x = PER_SCALE;
    point.scale.y = PER_SCALE;
    point.scale.z = PER_SCALE;

    // Make persistent point green
    point.color.r = 50.0 / 255.0;
    point.color.g = 1.0 / 2.0;
    point.color.b = 50.0 / 255.0;
    point.color.a = 1.0f;

    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = z;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    marker_per_pub_.publish(point);
  }

 private:
  ros::NodeHandle n;
  ros::Publisher marker_opp_in_;
  ros::Publisher marker_opp_out_pub_;
  ros::Publisher marker_per_pub_;

  unsigned int trail_counter;

};  // End of class SubscribeAndPublish

#endif
