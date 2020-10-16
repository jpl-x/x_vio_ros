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

#ifndef NODE_H_
#define NODE_H_

#include <x/vio/types.h>
#include <x/vio/vio.h>
#include <x_vio_ros/rviz_publisher.h>
#include <x_vio_ros/xvioConfig.h>

#include <image_transport/image_transport.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3Stamped.h> // Sun angle messages
#include <tf/transform_broadcaster.h>
//#include <sensor_fusion_comm/DoubleArrayStamped.h>
//#include <sensor_fusion_comm/DoubleMatrixStamped.h>
#include <geometry_msgs/TransformStamped.h>

#ifdef DUAL_THREAD
#include <boost/thread.hpp>
#endif

namespace x
{
//! XIO ROS node interface
class Node
{
public:
  Node(ros::NodeHandle& nh);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
  void visionImageCallback(const sensor_msgs::ImageConstPtr& img);
  void visionMatchesCallback(const std_msgs::Float64MultiArray::ConstPtr& matches_ptr);
  void rangeCallback(const sensor_msgs::RangeConstPtr& range_msg_ptr);
  void sunAngleCallback(const geometry_msgs::Vector3StampedConstPtr& sun_angle_msg_ptr);
  void processVision();
  void reconfigure(x_vio::xvioConfig& config, uint32_t level);

private:
  x::Params params_;                   ///< User parameters
  ros::Subscriber imu_sub_;              ///< Subscriber to IMU topic
  image_transport::Subscriber img_sub_;  ///< Subscriber to image topic
  ros::Subscriber matches_sub_;  ///< Subscriber to (visual) matches topic (e.g. output of a simulated feature tracker)
  ros::Subscriber range_sub_;            ///< Subscriber to range topic
  ros::Subscriber sun_angle_sub_;        ///< Subscriber to sun angle topic
  sensor_msgs::ImageConstPtr img_ptr_ = NULL;                 ///< Pointer to image data
  std_msgs::Float64MultiArray::ConstPtr matches_ptr_ = NULL;  ///< Pointer to matches data

  /**
   * Publisher for pose with covariance at image rate.
   */
  ros::Publisher pub_pose_with_cov_image_rate_;

#ifdef VERBOSE
  image_transport::Publisher tracker_pub_;       ///< Publishes tracker output image
  image_transport::Publisher track_manager_pub_; ///< Publishes track manager output image
  
  /**
   * Publisher for pose with covariance at IMU rate.
   */
  ros::Publisher pub_pose_with_cov_imu_rate_;

  ros::Publisher pub_tf_imu_rate_;               ///< Publishes 6DoF transform at IMU rate
  ros::Publisher pub_tf_image_rate_;             ///< Publishes 6DoF transform at image rate
  ros::Publisher pub_traj_imu_rate;              ///< Publishes trajectory at IMU rate
  ros::Publisher pub_traj_image_rate_;           ///< Publishes trajectory at image rate
  ros::Publisher pub_state_imu_rate_;            ///< Publishes state vector at IMU rate
  ros::Publisher pub_state_image_rate_;          ///< Publishes state vector at image rate

  /**
   * Publishes the covariance matrix at IMU rate.
   */
  ros::Publisher pub_cov_imu_rate_;

  ros::Publisher pub_cov_core_image_rate_;       ///< Publishes the covariance matrix for the core states at image rate
  rvizPublisher rviz_publisher_;
  mutable tf::TransformBroadcaster tf_broadcaster_;
#endif

  VIO vio_;                ///< XIO base instance
  unsigned int seq_ { 0 };  ///< \todo Remove this member and read frame_id directly from sim data.
#ifdef DUAL_THREAD
  bool new_img_ = false;  ///< True when a new image is waiting to be processed by the visual thread
  boost::mutex img_mutex_;
  boost::condition_variable img_ready_condition_;
#endif

  void loadParamsWithHandle(const ros::NodeHandle& nh);
  void setInterfaceWithHandle(ros::NodeHandle& nh);

  /**
   * Publish state content on ROS at image rate.
   *
   * @param[in] state Input state.
   */
  void publishImageRate(const State& state) const;

  void InitTrajMsgAtScale(visualization_msgs::Marker& msg, const double scale) const;
#ifdef VERBOSE
  /**
   * Publishes ROS messages at IMU rate.
   *
   * @param[in] state The state propagated with the last IMU message.
   */
  void publishImuRate(const State& state) const;

  /**
   * Broadcats a ROS TF transform from the input position and orientation.
   *
   * @param[in] p Position
   * @param[in] q Orientation unit quaternion.
   */
  void broadcastTf(const Vector3& p, const Quaternion& q) const;

  void publishSLAMFeatures(const State& state);

  void eigen2RosPoseCov(
      const Matrix& cov_eigen,
      geometry_msgs::PoseWithCovariance::_covariance_type& cov_ros) const;
#endif
};
}  // end namespace x

#endif  // NODE_H_
