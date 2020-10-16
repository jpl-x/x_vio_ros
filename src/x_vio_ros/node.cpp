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

#include <x_vio_ros/node.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace x;

namespace enc = sensor_msgs::image_encodings;

/** Set up xVIO within a ROS node.
 */
Node::Node(ros::NodeHandle& nh)
{
  // Load parameters from ROS
  loadParamsWithHandle(nh);

  // Set the estimator up with these parameters
  vio_.setUp(params_);

  // Set up inputs/outputs in ROS
  setInterfaceWithHandle(nh);

  // Initialize the filter at startup, if that parameter is enabled
  if (params_.init_at_startup)
  {
    ROS_INFO("Initializating xVIO");
    vio_.initAtTime(ros::Time::now().toSec());
  }
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr)
{
  // Stream IMU callback notification in Debug mode
#ifdef DEBUG
  ROS_INFO_STREAM("\033[1;33mIMU callback on core:\033[0m " << sched_getcpu()
               << ", thread: " << boost::this_thread::get_id()
               << " at timestamp: " << imu_msg_ptr->header.stamp);
#endif

  // Read accels
  Vector3 a_m(imu_msg_ptr->linear_acceleration.x,
              imu_msg_ptr->linear_acceleration.y,
              imu_msg_ptr->linear_acceleration.z);

  // Read gyros
  Vector3 w_m(imu_msg_ptr->angular_velocity.x,
              imu_msg_ptr->angular_velocity.y,
              imu_msg_ptr->angular_velocity.z);

  // Call xVIO IMU propagation
  const State propagated_state
    = vio_.processImu(imu_msg_ptr->header.stamp.toSec(),
                      imu_msg_ptr->header.seq,
                      w_m,
                      a_m);

#ifdef VERBOSE
  // Publish at IMU rate
  if (propagated_state.getTime() != kInvalid)
  {
    publishImuRate(propagated_state);
  }
#endif
}

/** \brief Image callback
 *
 * Handles threading and starts vision processing.
 */
void Node::visionImageCallback(const sensor_msgs::ImageConstPtr& img_ptr)
{
#ifdef DEBUG
  std::stringstream img_stamp;
  img_stamp << std::setprecision(20) << img_ptr->header.stamp;
  std::cout << "\033[1;31mImage callback on core:\033[0m" << sched_getcpu()
            << ", thread: " << boost::this_thread::get_id() << " at timestamp: " << img_stamp.str() << std::endl;
#endif

#ifdef DUAL_THREAD
  // store the new data for processing and wake the processing thread
  {
    boost::lock_guard<boost::mutex> lock(img_mutex_);
    new_img_ = true;
    img_ptr_ = img_ptr;
  }  // unlock the lock before waking the thread
  img_ready_condition_.notify_one();
#else
  img_ptr_ = img_ptr;
  processVision();
#endif
}

/** \brief Visual matches callback
 *
 * Handles threading and start vision processing.
 */
void Node::visionMatchesCallback(const std_msgs::Float64MultiArray::ConstPtr& matches_ptr)
{
#ifdef DEBUG
  std::cout << "\033[1;31mMatches callback on core:\033[0m" << sched_getcpu()
            << ", thread: " << boost::this_thread::get_id() << std::endl;
#endif

#ifdef DUAL_THREAD
  // store the new data for processing and wake the processing thread
  {
    boost::lock_guard<boost::mutex> lock(img_mutex_);
    new_img_ = true;
    matches_ptr_ = matches_ptr;
  }  // unlock the lock before waking the thread
  img_ready_condition_.notify_one();
#else
  matches_ptr_ = matches_ptr;
  processVision();
#endif
}

/** Range callback
 */
void Node::rangeCallback(const sensor_msgs::RangeConstPtr& range_msg_ptr)
{
  // Stream IMU callback notification in Debug mode
#if defined(DEBUG) && defined(VERBOSE)
  ROS_INFO_STREAM("\033[1;33mRange callback on core:\033[0m " << sched_getcpu()
                                                             << ", thread: " << boost::this_thread::get_id()
                                                             << " at timestamp: " << range_msg_ptr->header.stamp);
#endif

  // Initialize XIO range measurement
  RangeMeasurement range;
  range.timestamp = range_msg_ptr->header.stamp.toSec();
  range.range = range_msg_ptr->range;

  // Set it as last range measurement in xVIO
  vio_.setLastRangeMeasurement(range);
}

/** Sun angle callback
 */
void Node::sunAngleCallback(const geometry_msgs::Vector3StampedConstPtr& sun_angle_msg_ptr)
{
  // Stream sun angle callback notification in Debug mode
#if defined(DEBUG) && defined(VERBOSE)
  ROS_INFO_STREAM("\033[1;33mSun angle callback on core:\033[0m " << sched_getcpu()
                                                             << ", thread: " << boost::this_thread::get_id()
                                                             << " at timestamp: " << sun_angle_msg_ptr->header.stamp);
#endif

  // Initialize XIO sun angle measurement
  SunAngleMeasurement angle;
  angle.timestamp = sun_angle_msg_ptr->header.stamp.toSec();
  angle.x_angle = sun_angle_msg_ptr->vector.x;
  angle.y_angle = sun_angle_msg_ptr->vector.y;

  // Set it as last sun angle measurement in XIO
  vio_.setLastSunAngleMeasurement(angle);
}

/** \brief Common vision node interface
 *
 * When new vision data is ready to be processed, convert from ROS to xVIO
 * format and call xVIO vision measurement function.
 */
void Node::processVision()
{
#ifdef DUAL_THREAD
  while (ros::ok())
  {
    // Check there is indeed new data, otherwise sleep the thread
    boost::unique_lock<boost::mutex> lock(img_mutex_);

    while (!new_img_)
    {
      img_ready_condition_.wait(lock);
    }
#endif

    // Get data
    const sensor_msgs::ImageConstPtr img = img_ptr_;
    const std_msgs::Float64MultiArray::ConstPtr matches_ptr(matches_ptr_);

#ifdef DUAL_THREAD
    new_img_ = false;
#endif

    Time timestamp;
    TiledImage match_img, feature_img;
    State updated_state;
    if (img_ptr_ != NULL) // new image message received 
    {
#ifdef DEBUG
      std::stringstream img_stamp;
      img_stamp << std::setprecision(20) << img_ptr_->header.stamp;
      std::cout << "\033[1;31mStarting image processing on core:\033[0m" << sched_getcpu()
                << ", thread: " << boost::this_thread::get_id() << " at timestamp: " << img_stamp.str() << std::endl;
#endif

      // Initialize two image objects: both pass the raw image data. They will be
      // modified by the tracker and track manager to plot matches and features
      // types, respectively.
      timestamp = img->header.stamp.toSec();
      const unsigned int frame_number = img->header.seq;
      cv_bridge::CvImageConstPtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvShare(img, enc::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Shallow copies
      match_img = TiledImage(cv_ptr->image, timestamp, frame_number,
                                params_.n_tiles_h,
                                params_.n_tiles_w,
                                params_.max_feat_per_tile);
      feature_img = TiledImage(match_img);

      updated_state = vio_.processImageMeasurement(timestamp,
                                                   img->header.seq,
                                                   match_img,
                                                   feature_img);

#ifdef DEBUG
      std::cout << "\033[1;31mEnding image processing on core:\033[0m" << sched_getcpu()
                << ", thread: " << boost::this_thread::get_id() << " at timestamp: " << img_stamp.str() << std::endl;
#endif
    }
    else // new match message received
    {
      // Increment seq ID
      ++seq_;

      // Read timestamp
      timestamp = matches_ptr->data[3];

#ifdef DEBUG
      std::cout << "\033[1;31mStarting matches processing on core:\033[0m" << sched_getcpu()
        << ", thread: " << boost::this_thread::get_id() << " at timestamp: " << timestamp << std::endl;
#endif

      // Initialize plain image to plot features on
      cv::Mat cv_match_img(params_.img_height,
                           params_.img_width,
                           CV_8UC1,
                           cv::Scalar(0));
      match_img = TiledImage(cv_match_img,
                                timestamp,
                                seq_,
                                params_.n_tiles_h,
                                params_.n_tiles_w,
                                params_.max_feat_per_tile);
      feature_img = TiledImage(match_img);

      // Convert ROS match message to std::vector (no ROS allowed beyond the wrapper)
      std::vector<double> matches = matches_ptr->data;

      // Pass matches to VIO
      updated_state = vio_.processMatchesMeasurement(timestamp,
                                                     seq_,
                                                     matches,
                                                     match_img,
                                                     feature_img);
    }

    // If the update has been successfully applied in the filter
    if (updated_state.getTime() != kInvalid)
    {
      // Publish state
      publishImageRate(updated_state);

#ifdef VERBOSE
      // Publish images to be displayed in the GUI
      const sensor_msgs::ImagePtr match_img_msg =
          cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, match_img).toImageMsg();
      tracker_pub_.publish(match_img_msg);

      const sensor_msgs::ImagePtr feature_img_msg =
          cv_bridge::CvImage(std_msgs::Header(), enc::BGR8, feature_img).toImageMsg();
      track_manager_pub_.publish(feature_img_msg);

	    // Publish 3D features
      publishSLAMFeatures(updated_state);
      Vector3dArray msckf_inliers, msckf_outliers;
      vio_.getMsckfFeatures(msckf_inliers, msckf_outliers);
      rviz_publisher_.publishMsckfFeatures(msckf_inliers, msckf_outliers);
#endif
    }
#ifdef DUAL_THREAD
  }
#endif
}

/** Reconfigure callback
 */
void Node::reconfigure(x_vio::xvioConfig& config, uint32_t level)
{
  if (config.INITIALIZE_FILTER)
  {
    ROS_INFO("xVIO init request");
    // (Re-)initialization should happen after image processing has
    // returned since it is using pointers to the state vector
#ifdef DUAL_THREAD
    boost::lock_guard<boost::mutex> lock(img_mutex_);
#endif
    vio_.initAtTime(ros::Time::now().toSec());
    config.INITIALIZE_FILTER = 0;
  }
}

/** \brief Load user parameters from ROS parameter server.
 *
 *  Parameters stored in params.yaml have been loaded on the ROS parameter
 *  server by the launch file. They are accessed with the node handle.
 */
void Node::loadParamsWithHandle(const ros::NodeHandle& nh)
{
  // Eigen is not supported by ROS' getParam function so we import these
  // variable as std::vectors first
  std::vector<double> p, v, b_w, b_a, q, p_ic, q_ic, q_sc, w_s;

  // Import initial state or kill xVIO
  bool vio_lives = true;
  vio_lives = vio_lives && nh.getParam("/x_vio/p", p);
  vio_lives = vio_lives && nh.getParam("/x_vio/v", v);
  vio_lives = vio_lives && nh.getParam("/x_vio/q", q);
  vio_lives = vio_lives && nh.getParam("/x_vio/b_w", b_w);
  vio_lives = vio_lives && nh.getParam("/x_vio/b_a", b_a);

  if (!vio_lives)
    ROS_ERROR("Initial state parameters are missing!");

  // Import camera calibration or kill xVIO.
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_fx", params_.cam_fx);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_fy", params_.cam_fy);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_cx", params_.cam_cx);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_cy", params_.cam_cy);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_s", params_.cam_s);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_img_height", params_.img_height);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_img_width", params_.img_width);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_p_ic", p_ic);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_q_ic", q_ic);
  vio_lives = vio_lives && nh.getParam("/x_vio/cam1_time_offset", params_.time_offset);
  vio_lives = vio_lives && nh.getParam("/x_vio/sigma_img", params_.sigma_img);

  if (!vio_lives)
    ROS_ERROR("Camera parameters are missing!");

  // Laser range finder
  vio_lives = vio_lives && nh.getParam("/x_vio/sigma_range", params_.sigma_range);

  if (!vio_lives)
    ROS_ERROR("Laser range finder parameters are missing!");

  // Import sun sensor calibration or kill xVIO.
  vio_lives = vio_lives && nh.getParam("/x_vio/q_sc", q_sc);
  vio_lives = vio_lives && nh.getParam("/x_vio/w_s", w_s);

  if (!vio_lives)
    ROS_ERROR("Sun sensor parameters are missing!");

  // Import IMU parameters or kill xVIO
  vio_lives = vio_lives && nh.getParam("/x_vio/n_a", params_.n_a);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_ba", params_.n_ba);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_w", params_.n_w);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_bw", params_.n_bw);

  if (!vio_lives)
    ROS_ERROR("IMU parameters are missing!");

  // Import visual front end parameters or kill xVIO
  vio_lives = vio_lives && nh.getParam("/x_vio/fast_detection_delta", params_.fast_detection_delta);
  vio_lives = vio_lives && nh.getParam("/x_vio/non_max_supp", params_.non_max_supp);
  vio_lives = vio_lives && nh.getParam("/x_vio/block_half_length", params_.block_half_length);
  vio_lives = vio_lives && nh.getParam("/x_vio/margin", params_.margin);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_feat_min", params_.n_feat_min);
  vio_lives = vio_lives && nh.getParam("/x_vio/outlier_method", params_.outlier_method);
  vio_lives = vio_lives && nh.getParam("/x_vio/outlier_param1", params_.outlier_param1);
  vio_lives = vio_lives && nh.getParam("/x_vio/outlier_param2", params_.outlier_param2);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_tiles_h", params_.n_tiles_h);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_tiles_w", params_.n_tiles_w);
  vio_lives = vio_lives && nh.getParam("/x_vio/max_feat_per_tile", params_.max_feat_per_tile);

  if (!vio_lives)
    ROS_ERROR("Visual front end parameters are missing!");

  // Import filter parameters or kill xVIO
  vio_lives = vio_lives && nh.getParam("/x_vio/n_poses_max", params_.n_poses_max);
  vio_lives = vio_lives && nh.getParam("/x_vio/n_slam_features_max", params_.n_slam_features_max);
  vio_lives = vio_lives && nh.getParam("/x_vio/rho_0", params_.rho_0);
  vio_lives = vio_lives && nh.getParam("/x_vio/sigma_rho_0", params_.sigma_rho_0);
  vio_lives = vio_lives && nh.getParam("/x_vio/iekf_iter", params_.iekf_iter);
  vio_lives = vio_lives && nh.getParam("/x_vio/msckf_baseline", params_.msckf_baseline);
  vio_lives = vio_lives && nh.getParam("/x_vio/min_track_length", params_.min_track_length);
  vio_lives = vio_lives && nh.getParam("/x_vio/traj_timeout_gui", params_.traj_timeout_gui);
  vio_lives = vio_lives && nh.getParam("/x_vio/init_at_startup", params_.init_at_startup);

  if (!vio_lives)
    ROS_ERROR("Filter parameters are missing!");

  // Convert std::vectors to msc vectors and quaternions in params
  params_.p << p[0], p[1], p[2];
  params_.v << v[0], v[1], v[2];
  params_.q.w() = q[0];
  params_.q.x() = q[1];
  params_.q.y() = q[2];
  params_.q.z() = q[3];
  params_.q.normalize();
  params_.b_w << b_w[0], b_w[1], b_w[2];
  params_.b_a << b_a[0], b_a[1], b_a[2];
  params_.p_ic << p_ic[0], p_ic[1], p_ic[2];
  params_.q_ic.w() = q_ic[0];
  params_.q_ic.x() = q_ic[1];
  params_.q_ic.y() = q_ic[2];
  params_.q_ic.z() = q_ic[3];
  params_.q_ic.normalize();
  params_.q_sc.w() = q_sc[0];
  params_.q_sc.x() = q_sc[1];
  params_.q_sc.y() = q_sc[2];
  params_.q_sc.z() = q_sc[3];
  params_.q_sc.normalize();
  params_.w_s << w_s[0], w_s[1], w_s[2];
  params_.w_s.normalize();
}

/** Set xVIO subscribers/publishers for ROS.
 */
void Node::setInterfaceWithHandle(ros::NodeHandle& nh)
{
  image_transport::ImageTransport it(nh);

  /***************
   * Subscribers *
   ***************/
  // Note: only one of the vision subscribers should actually be receiving data
  // during a given run, either images or matches. Although nothing technically
  // prevents the processing of both in the code, something has most likely
  // been set incorrectly by the user if this is happening.

  imu_sub_ = nh.subscribe("imu", 100, &Node::imuCallback, this);
  img_sub_ = it.subscribe("image_raw", 2, &Node::visionImageCallback, this);
  matches_sub_ = nh.subscribe("features", 2, &Node::visionMatchesCallback, this);
  range_sub_ = nh.subscribe("range", 2, &Node::rangeCallback, this);
  sun_angle_sub_ = nh.subscribe("sun_angles", 2, &Node::sunAngleCallback, this);

  /**************
   * Publishers *
   **************/

  pub_pose_with_cov_image_rate_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_with_cov_image_rate", 2);

#ifdef VERBOSE
  pub_pose_with_cov_imu_rate_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_with_cov_imu_rate", 2);
  tracker_pub_ = it.advertise("tracker", 1);
  track_manager_pub_ = it.advertise("track_manager", 1);
  pub_tf_imu_rate_ = nh.advertise<geometry_msgs::TransformStamped>("tf_imu_rate", 2);
  pub_tf_image_rate_ = nh.advertise<geometry_msgs::TransformStamped>("tf_image_rate", 2);
  pub_traj_imu_rate = nh.advertise<visualization_msgs::Marker>("traj_imu_rate", 2);
  pub_traj_image_rate_ = nh.advertise<visualization_msgs::Marker>("traj_image_rate", 2);
  //pub_state_image_rate_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped>("state_image_rate", 2);
  //pub_state_imu_rate_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped>("state_imu_rate", 2);
  //pub_cov_imu_rate_ = nh.advertise<sensor_fusion_comm::DoubleMatrixStamped>("cov_imu_rate", 2);
  //pub_cov_core_image_rate_ = nh.advertise<sensor_fusion_comm::DoubleMatrixStamped>("cov_core_image_rate", 2);
#endif
  rviz_publisher_.setUpPublishers(params_.n_poses_max,
                                  params_.n_slam_features_max);

  // Print published/subscribed topics.
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string nodeName = ros::this_node::getName();
  std::string topicsStr = "\n\t" + nodeName + ":\n\tsubscribed topics:\n";
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  ROS_INFO_STREAM(topicsStr);
}

#ifdef VERBOSE
void Node::publishImuRate(const State& state) const
{
  // Parse state
  const double t = state.getTime();
  const Vector3 p = state.getPosition();
  const Quaternion q = state.getOrientation();
  const Matrix& pose_cov = state.getPoseCovariance();

  // Just compute the header once for all messages
  // WARNING: header's seq is overwritten by ROS. There is no point in setting
  // it here.
  geometry_msgs::PoseWithCovarianceStamped msg_pose;
  msg_pose.header.stamp = ros::Time(t);
  msg_pose.header.frame_id = "world";
  msg_pose.pose.pose.position.x = p(0);
  msg_pose.pose.pose.position.y = p(1);
  msg_pose.pose.pose.position.z = p(2);
  msg_pose.pose.pose.orientation.w = q.w();
  msg_pose.pose.pose.orientation.x = q.x();
  msg_pose.pose.pose.orientation.y = q.y();
  msg_pose.pose.pose.orientation.z = q.z();
  eigen2RosPoseCov(pose_cov, msg_pose.pose.covariance);

  // Publishing
  if (pub_pose_with_cov_imu_rate_.getNumSubscribers()) // pose
    pub_pose_with_cov_imu_rate_.publish(msg_pose);

  if(pub_tf_imu_rate_.getNumSubscribers()) // transform
  {
    geometry_msgs::TransformStamped msg_tf;
    msg_tf.header = msg_pose.header;
    msg_tf.child_frame_id = "xVIO";
    msg_tf.transform.translation.x = p(0);
    msg_tf.transform.translation.y = p(1);
    msg_tf.transform.translation.z = p(2);
    msg_tf.transform.rotation.w = q.w();
    msg_tf.transform.rotation.x = q.x();
    msg_tf.transform.rotation.y = q.y();
    msg_tf.transform.rotation.z = q.z();
    pub_tf_imu_rate_.publish(msg_tf);
  }

  static int msg_seq_imu = 0;
  if(pub_traj_imu_rate.getNumSubscribers()) // trajectory
  {
    visualization_msgs::Marker msg_traj;
    InitTrajMsgAtScale(msg_traj, 0.008);
    msg_traj.header = msg_pose.header;
	msg_traj.id = msg_seq_imu++;
    msg_traj.pose.position = msg_pose.pose.pose.position;
    msg_traj.pose.orientation = msg_pose.pose.pose.orientation;
    msg_traj.lifetime = ros::Duration(params_.traj_timeout_gui);
    pub_traj_imu_rate.publish(msg_traj);
  }

  //TODO(jeff) Do xEKF state and cov getters and publishers
/*
  if(pub_state_imu_rate_.getNumSubscribers()) // state
  {
    sensor_fusion_comm::DoubleArrayStamped msg_state;
    msg_state.header = msg_pose.header;
    msf_core::msf_double_array_type msf_double_array;
    state->ToFullStateStruct(msf_double_array);
    unsigned int n = msf_double_array.data.size();
    msg_state.data.resize(n);
    for (unsigned i=0; i<n; i++)
        msg_state.data[i] = msf_double_array.data[i];
    pub_state_imu_rate_.publish(msg_state);
  }
*/
  // Covariance
  if(pub_cov_imu_rate_.getNumSubscribers())
  {
    //sensor_fusion_comm::DoubleMatrixStampedPtr msg(new sensor_fusion_comm::DoubleMatrixStamped);
    //msg->header = msg_pose.header;
    //msf_core::msf_double_matrix_type cov;
    //state->GetCoreCovariance(cov);
    //msg->cols = cov.cols;
    //msg->rows = cov.rows;
    //unsigned int n_core = cov.data.size();
    //msg->data.resize(n_core);
    //for (unsigned int i = 0; i < n_core; i++) {
    //    msg->data[i] = cov.data[i];
    //}
    //pub_cov_core_imu_rate_.publish(msg);
  }

  // TF broadcast
  broadcastTf(p, q);
}

void Node::broadcastTf(const Vector3& p, const Quaternion& q) const
{
  tf::Transform transform;
  geometry_msgs::TransformStamped msgTf;
  msgTf.transform.translation.x = p(0);
  msgTf.transform.translation.y = p(1);
  msgTf.transform.translation.z = p(2);
  msgTf.transform.rotation.w = q.w();
  msgTf.transform.rotation.x = q.x();
  msgTf.transform.rotation.y = q.y();
  msgTf.transform.rotation.z = q.z();

  const geometry_msgs::Vector3& pos = msgTf.transform.translation;
  const geometry_msgs::Quaternion& ori = msgTf.transform.rotation;
  transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
  transform.setRotation(tf::Quaternion(ori.x, ori.y, ori.z, ori.w));
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "xVIO"));
}

/** Publishes the SLAM features in rviz.
 */
void Node::publishSLAMFeatures(const State& state)
{
  // Delete all persistent features previously published in rviz
  // (Avoid tracking publishing IDs, make it clever if necessary)
  rviz_publisher_.deleteAllPersistentFeatures();

  const std::vector<Eigen::Vector3d> features = vio_.computeSLAMCartesianFeaturesForState(state);

  // Publish in rviz
  for (unsigned int i = 0; i < features.size(); i++)
  {
    const Eigen::Vector3d feature = features[i];
    rviz_publisher_.publishPersistentFeature(feature(0), feature(1), feature(2), i);
  }
}
#endif

void Node::publishImageRate(const State& state) const
{
  // Parse state
  const double t = state.getTime();
  const Vector3 p = state.getPosition();
  const Quaternion q = state.getOrientation();
  const Matrix& pose_cov = state.getPoseCovariance();

  // Just compute the header once for all messages
  // WARNING: header's seq is overwritten by ROS. There is no point in setting
  // it here.
  geometry_msgs::PoseWithCovarianceStamped msg_pose;
  msg_pose.header.stamp = ros::Time(t);
  msg_pose.header.frame_id = "world";
  msg_pose.pose.pose.position.x = p(0);
  msg_pose.pose.pose.position.y = p(1);
  msg_pose.pose.pose.position.z = p(2);
  msg_pose.pose.pose.orientation.w = q.w();
  msg_pose.pose.pose.orientation.x = q.x();
  msg_pose.pose.pose.orientation.y = q.y();
  msg_pose.pose.pose.orientation.z = q.z();
  eigen2RosPoseCov(pose_cov, msg_pose.pose.covariance);

  // Publishing   Pose @ image rate
  if(pub_pose_with_cov_image_rate_.getNumSubscribers()) // pose
    pub_pose_with_cov_image_rate_.publish(msg_pose);
#ifdef VERBOSE
  // Publishing   tf @ image rate
  if(false)//pub_tf_image_rate_.getNumSubscribers()) // transform
  {
    geometry_msgs::TransformStamped msg_tf;
    msg_tf.header = msg_pose.header;
    msg_tf.child_frame_id = "xVIO";
    msg_tf.transform.translation.x = p(0);
    msg_tf.transform.translation.y = p(1);
    msg_tf.transform.translation.z = p(2);
    msg_tf.transform.rotation.w = q.w();
    msg_tf.transform.rotation.x = q.x();
    msg_tf.transform.rotation.y = q.y();
    msg_tf.transform.rotation.z = q.z();
    pub_tf_image_rate_.publish(msg_tf);
  }

  static int msg_seq_img = 0;
  // Publishing traj @ image rate
  if (pub_traj_image_rate_.getNumSubscribers()) // trajectory
  {
    visualization_msgs::Marker msg_traj;
    InitTrajMsgAtScale(msg_traj, 0.02);
    msg_traj.header = msg_pose.header;
    msg_traj.id = msg_seq_img++;
    msg_traj.pose.position = msg_pose.pose.pose.position;
    msg_traj.pose.orientation = msg_pose.pose.pose.orientation;
    msg_traj.lifetime = ros::Duration(params_.traj_timeout_gui);
    pub_traj_image_rate_.publish(msg_traj);
  }
/*
  // Publishing   State @ image rate
  if(pub_state_image_rate_.getNumSubscribers()) // state
  {
    sensor_fusion_comm::DoubleArrayStamped msg_state;
    msg_state.header = msg_pose.header;

    msf_core::msf_double_array_type msf_double_array;
    state->ToFullStateStruct(msf_double_array);
    unsigned int n = msf_double_array.data.size();
    msg_state.data.resize(n);
    for (unsigned i=0; i<n; i++)
        msg_state.data[i] = msf_double_array.data[i];
    pub_state_imu_rate_.publish(msg_state);

    pub_state_image_rate_.publish(msg_state);
  }

  // Publishing   cov_core @ image rate
  if (pub_cov_core_image_rate_.getNumSubscribers()) // covariance core block
  {
    sensor_fusion_comm::DoubleMatrixStampedPtr msg(new sensor_fusion_comm::DoubleMatrixStamped);
    msg->header = msg_pose.header;
    msf_core::msf_double_matrix_type cov;
    state->GetCoreCovariance(cov);
    msg->cols = cov.cols;
    msg->rows = cov.rows;
    unsigned int n_core = cov.data.size();
    msg->data.resize(n_core);
    for (unsigned int i = 0; i < n_core; i++) {
        msg->data[i] = cov.data[i];
    }
    pub_cov_core_image_rate_.publish(msg);
  }
*/
  // TF broadcast
  broadcastTf(p, q);
#endif
}

/** Initializes a trajectory marker message at a given scale.
 */
void Node::InitTrajMsgAtScale(visualization_msgs::Marker& msg, const double scale) const
{
  msg.ns = "trajectory";
  msg.type = visualization_msgs::Marker::CUBE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = scale;
  msg.scale.y = scale;
  msg.scale.z = scale;
  msg.color.a = 1.0;
  msg.color.r = 0.;  // blue
  msg.color.g = 0.;
  msg.color.b = 0.5;
  msg.lifetime = ros::Duration(0.0);
}

void Node::eigen2RosPoseCov(
    const Matrix& cov_eigen,
    geometry_msgs::PoseWithCovariance::_covariance_type& cov_ros) const {
  
  for(int r=0; r<6; r++) {
    for(int c=0; c<6; c++) {
      cov_ros[6*r+c] = cov_eigen(r,c);
    }
  }
}
