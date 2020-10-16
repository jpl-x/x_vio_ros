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

// Bring in my package's API, which is what I'm testing
#include <x/vio/types.h>
#include <x/vio/vio.h>
// Bring in ros to subscribe to topics
#include <ros/ros.h>
#include <rosbag/bag.h>
// Bring in sensor message types we will be testing
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
// Bring in gtest
#include <gtest/gtest.h>
#include <math.h>
#include <thread>
#include <atomic>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;
geometry_msgs::PoseWithCovarianceStampedConstPtr est_pose_;
tf::StampedTransform true_pose_transform_;

void est_poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  est_pose_ = msg;
}

// Test position accuracy
TEST(TestSuite, positionAccuracy)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  double tolerance = 0.0;
  std::string groundtruth_parent_frame;
  std::string groundtruth_child_frame;
  std::string xvio_topic;

  nh_local.getParam("groundtruth_parent_frame", groundtruth_parent_frame);
  nh_local.getParam("groundtruth_child_frame", groundtruth_child_frame);
  nh_local.getParam("tolerance", tolerance);

  ros::Subscriber sub = nh.subscribe("/x_vio/pose_image_rate",1, &est_poseCallback);

  tf::TransformListener true_poseListener;
  true_poseListener.waitForTransform(groundtruth_parent_frame, groundtruth_child_frame, ros::Time(0), ros::Duration(5.0));

  while(ros::ok()){
    ros::spinOnce();
    true_poseListener.lookupTransform(groundtruth_parent_frame, groundtruth_child_frame, ros::Time(0), true_pose_transform_);
  }

  tf::Vector3 tf_vec;
  tf_vec = true_pose_transform_.getOrigin();

  double x_diff = tf_vec.getX()-est_pose_->pose.pose.position.x;
  double y_diff = tf_vec.getY()-est_pose_->pose.pose.position.y;
  double z_diff = tf_vec.getZ()-est_pose_->pose.pose.position.z;

  EXPECT_LE(sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff),tolerance);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "x_vioTest");
  ros::Time::init();
  
  auto res = RUN_ALL_TESTS();
  
  return res;
}
