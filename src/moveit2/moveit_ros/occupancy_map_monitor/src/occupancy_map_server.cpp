/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <octomap_msgs/conversions.h>
#include <boost/bind.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.occupancy_map_server");

static void publishOctomap(const rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr& octree_binary_pub,
                           occupancy_map_monitor::OccupancyMapMonitor* server)
{
  octomap_msgs::msg::Octomap map;

  map.header.frame_id = server->getMapFrame();
  map.header.stamp = rclcpp::Clock().now();

  server->getOcTreePtr()->lockRead();
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  try
  {
    if (!octomap_msgs::binaryMapToMsgData(*server->getOcTreePtr(), map.data))
      RCLCPP_ERROR_THROTTLE(LOGGER, steady_clock, 1000, "Could not generate OctoMap message");
  }
  catch (...)
  {
    RCLCPP_ERROR_THROTTLE(LOGGER, steady_clock, 1000, "Exception thrown while generating OctoMap message");
  }
  server->getOcTreePtr()->unlockRead();

  octree_binary_pub->publish(map);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("occupancy_map_server");
  auto octree_binary_pub =
      node->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  auto clock_ptr = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(clock_ptr, tf2::durationFromSec(5.0));
  std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*buffer, node);
  occupancy_map_monitor::OccupancyMapMonitor server(node, buffer);
  server.setUpdateCallback(boost::bind(&publishOctomap, octree_binary_pub, &server));
  server.startMonitor();

  rclcpp::spin(node);
  return 0;
}
