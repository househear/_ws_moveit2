/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Author: Boston Cleek
   Desc: A simple demo node running ompl constrained planning capabilities for planning and execution
*/
#include <memory>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <warehouse_ros_mongo/database_connection.h>
#include "rock_rhino_database/visibility_control.h"
namespace gm = geometry_msgs;
using std::cout;
using std::string;
using std::vector;
using warehouse_ros::Metadata;
using warehouse_ros::NoMatchingMessageException;
using warehouse_ros::Query;

typedef warehouse_ros::MessageCollection<gm::msg::Pose> PoseCollection;
typedef warehouse_ros::MessageWithMetadata<gm::msg::Pose> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
Metadata::Ptr makeMetadata(const PoseCollection& coll, const gm::msg::Pose& p, const string& n)
{
  Metadata::Ptr meta = coll.createMetadata();
  meta->append("x", p.position.x);
  meta->append("y", p.position.y);
  meta->append("name", n);
  return meta;
}


static const rclcpp::Logger LOGGER = rclcpp::get_logger("ompl_constrained_planning_demo");
static const std::string PLANNING_GROUP = "panda_arm";
static const double PLANNING_TIME_S = 30.0;
static const double PLANNING_ATTEMPTS = 5.0;

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;



inline geometry_msgs::msg::Pose makePose(const double x, const double y, const double theta)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  return p;
}


namespace test_demo
{
class DB_Manager : public rclcpp::Node
{
  public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ROCK_RHINO_DATABASE_PUBLIC
  explicit DB_Manager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("db_manager", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&DB_Manager::handle_goal, this, _1, _2),
      std::bind(&DB_Manager::handle_cancel, this, _1),
      std::bind(&DB_Manager::handle_accepted, this, _1));
  }

  private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  ROCK_RHINO_DATABASE_LOCAL
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  ROCK_RHINO_DATABASE_LOCAL
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  ROCK_RHINO_DATABASE_LOCAL
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DB_Manager::execute, this, _1), goal_handle}.detach();
  }

  ROCK_RHINO_DATABASE_LOCAL
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }


};

}

RCLCPP_COMPONENTS_REGISTER_NODE(test_demo::DB_Manager)
