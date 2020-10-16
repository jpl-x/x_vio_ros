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

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

 /** Run (multithreaded) XIO within a ROS node
 */
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "x_vio");

  // Start node
  ros::NodeHandle nh("x_vio");
  
  // Set up xVIO within this node
  x::Node vio_node(nh);

  // Set up visual thread
  #ifdef DUAL_THREAD
  boost::thread visual_thread = boost::thread(boost::bind(&x::Node::processVision, &vio_node));
  #endif

  // Set up dynamic reconfigure
  dynamic_reconfigure::Server<x_vio::xvioConfig> srv; 
  dynamic_reconfigure::Server<x_vio::xvioConfig>::CallbackType f;
  f = boost::bind(&x::Node::reconfigure, &vio_node, _1, _2);
  srv.setCallback(f);

  // Cycle callbacks
  ros::spin();

  // Terminate visual thread
  #ifdef DUAL_THREAD
  // Interrupt callback thread, otherwise node closing will hang, since join
  // waits for the thread to finish its work.
  visual_thread.interrupt();
  visual_thread.join();
  #endif

  return 0;
}
