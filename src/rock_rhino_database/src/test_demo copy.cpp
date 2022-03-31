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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <warehouse_ros_mongo/database_connection.h>

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


class DB_Manager
{
  public:
    bool run_on = false;
    DB_Manager(const rclcpp::Node::SharedPtr& node): node_(node)
    {
      //init db_manager
      ref_link_ = 'abc';
    
      
    }

  private:
    rclcpp::Node::SharedPtr node_;
    std::string ref_link_, ee_link_;
}

inline geometry_msgs::msg::Pose makePose(const double x, const double y, const double theta)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  return p;
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
        // This enables loading undeclared parameters
        // best practice would be to declare parameters in the corresponding classes
        // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("db_manager", node_options);
  DB_Manager db_manager(node);
  warehouse_ros_mongo::MongoDatabaseConnection conn;
  conn.setParams("localhost", 33829, 60.0);
  conn.connect();
  if (conn.isConnected())
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "connection succefull");
  };



  // Clear existing data if any
  conn.dropDatabase("my_db");


  // Open the collection
  PoseCollection coll = conn.openCollection<gm::msg::Pose>("my_db", "poses");

  // Arrange to index on metadata fields 'x' and 'name'
  // coll.ensureIndex("name");
  // coll.ensureIndex("x");

  // Add some poses and metadata
  const gm::msg::Pose p1 = makePose(24, 42, 0);
  const gm::msg::Pose p2 = makePose(10, 532, 3);
  const gm::msg::Pose p3 = makePose(53, 22, 5);
  const gm::msg::Pose p4 = makePose(22, -5, 33);


  coll.insert(p1, makeMetadata(coll, p1, "bar"));
  coll.insert(p2, makeMetadata(coll, p2, "baz"));
  coll.insert(p3, makeMetadata(coll, p3, "qux"));
  coll.insert(p1, makeMetadata(coll, p1, "oof"));
  coll.insert(p4, makeMetadata(coll, p4, "ooof"));


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sssssssss");

  Query::Ptr q1 = coll.createQuery();
    q1->append("name", "qux");
    vector<PoseMetaPtr> res = coll.queryList(q1, true);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return: %f", res[0]->lookupDouble("x"));


  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


