// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <cstdio>
#include <memory>

#include "rock_rhino_msgs/action/process_command.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "db_test/visibility_control.h"

#include <geometry_msgs/msg/pose.hpp>
#include <warehouse_ros_mongo/database_connection.h>
#include "rock_rhino_msgs/msg/detected_tag.hpp"
namespace gm = geometry_msgs;
using std::cout;
using std::string;
using std::vector;
using std::to_string;
using warehouse_ros::Metadata;
using warehouse_ros::NoMatchingMessageException;
using warehouse_ros::Query;

typedef warehouse_ros::MessageCollection<rock_rhino_msgs::msg::DetectedTag> PoseCollection;
typedef warehouse_ros::MessageWithMetadata<rock_rhino_msgs::msg::DetectedTag> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
Metadata::Ptr makeMetadata(const PoseCollection& coll, const int& n)
{
  Metadata::Ptr meta = coll.createMetadata();
  meta->append("ID", n);
  meta->append("status", 11);
  return meta;
}


// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
Metadata::Ptr _makeMetadata(const PoseCollection& coll, const int& n)
{
  Metadata::Ptr meta = coll.createMetadata();
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

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = cos(yaw / 2);
  q.z = sin(yaw / 2);
  q.x = 0;
  q.y = 0;
  return q;
}
//const std::shared_ptr<rock_rhino_msgs::msg::DetectedTag> msg
inline rock_rhino_msgs::msg::DetectedTag _makePose(const int tag_id, const double x)
{
  rock_rhino_msgs::msg::DetectedTag p;
  p.tag_id = tag_id;
  p.pose.position.x = x;

  return p;
}

namespace db_test
{
class ProCtrl2DataBaseActionServer : public rclcpp::Node
{
public:
  using ProcessCommand = rock_rhino_msgs::action::ProcessCommand;
  using GoalHandleProcessCommand = rclcpp_action::ServerGoalHandle<ProcessCommand>;

  DB_TEST_PUBLIC
  explicit ProCtrl2DataBaseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("data_base", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ProcessCommand>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "data_base/action_server_pro_ctr",
      std::bind(&ProCtrl2DataBaseActionServer::handle_goal, this, _1, _2),
      std::bind(&ProCtrl2DataBaseActionServer::handle_cancel, this, _1),
      std::bind(&ProCtrl2DataBaseActionServer::handle_accepted, this, _1));


      //start: listening image_processor/detected_tag
      setvbuf(stdout, NULL, _IONBF, BUFSIZ);
      auto callback =
        [this](const std::shared_ptr<rock_rhino_msgs::msg::DetectedTag> msg) -> void
        {
          //RCLCPP_INFO(this->get_logger(), "I heard: [%d]", msg->tag_id);
          //check if this tag exits
          //update_db(msg);
          
        };

      sub_ = create_subscription<rock_rhino_msgs::msg::DetectedTag>("image_processor/detected_tag", rclcpp::SensorDataQoS(), callback);
    
      //end: listening image_processor/detected_tag   

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
      coll = conn.openCollection<rock_rhino_msgs::msg::DetectedTag>("my_db", "poses");

      // Arrange to index on metadata fields 'x' and 'name'
      // coll.ensureIndex("name");
      // coll.ensureIndex("x");


  }

private:
  rclcpp_action::Server<ProcessCommand>::SharedPtr action_server_;
  rclcpp::Subscription<rock_rhino_msgs::msg::DetectedTag>::SharedPtr sub_;
  PoseCollection coll;


  DB_TEST_LOCAL
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ProcessCommand::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  DB_TEST_LOCAL
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleProcessCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  DB_TEST_LOCAL
  void handle_accepted(const std::shared_ptr<GoalHandleProcessCommand> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ProCtrl2DataBaseActionServer::execute, this, _1), goal_handle}.detach();
  }

  DB_TEST_LOCAL
  void execute(const std::shared_ptr<GoalHandleProcessCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ProcessCommand::Feedback>();
    auto & sequence = feedback->sequence;
    unsigned q1 = 0;
    auto result = std::make_shared<ProcessCommand::Result>();
    if (goal->task_name == "query1") {
         RCLCPP_INFO(this->get_logger(), "Executing t1"); //inqury the data base if it is empty
         q1 = coll.count();
         sequence.push_back(q1);
         sequence.push_back(2);
         sequence.push_back(3);
          if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
          }
    }

    if (goal->task_name == "query") {
         RCLCPP_INFO(this->get_logger(), "querying db"); //inqury the data base if it is empty
         query_all();
    }    

    if (goal->task_name == "add") {
         RCLCPP_INFO(this->get_logger(), "add record into db"); //inqury the data base if it is empty
         add_record();
    }    

    if (goal->task_name == "delete") {
        RCLCPP_INFO(this->get_logger(), "delete record in db"); //inqury the data base if it is empty
        delete_record();
}    
  }


  void add_record() //print out all records in db
  {

    const rock_rhino_msgs::msg::DetectedTag dg1 = _makePose(11, 42.3);
    coll.insert(dg1, _makeMetadata(coll, dg1.tag_id));

    const rock_rhino_msgs::msg::DetectedTag dg2 = _makePose(15, 4.3);
    coll.insert(dg2, _makeMetadata(coll, dg2.tag_id));
    // const gm::msg::Pose p1 = _makePose(24, 42, 0);
    // const gm::msg::Pose p2 = _makePose(10, 532, 3);
    // const gm::msg::Pose p3 = _makePose(53, 22, 5);
    // const gm::msg::Pose p4 = _makePose(22, -5, 33);
    // coll.insert(p1, _makeMetadata(coll, p1, "bar"));
    // coll.insert(p2, _makeMetadata(coll, p2, "baz"));
    // coll.insert(p3, _makeMetadata(coll, p3, "qux"));
    // coll.insert(p1, _makeMetadata(coll, p1, "oof"));
    // coll.insert(p4, _makeMetadata(coll, p4, "ooof"));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return: %d", coll.count());
  }

  void delete_record() //print out all records in db
  {

      //delete a record
    Query::Ptr q3 = coll.createQuery();
    q3->append("name", 11);
    coll.removeMessages(q3);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return: %d", coll.count());



  // // Set up query to delete some poses.
  // Query::Ptr q3 = coll.createQuery();
  // q3->appendLT("y", 30);

  // EXPECT_EQ(5u, coll.count());
  // EXPECT_EQ(2u, coll.removeMessages(q3));
  // EXPECT_EQ(3u, coll.count());
  }

  void query_all() //
  {

    // Simple query: find the pose with name 'qux' and return just its metadata
    // Since we're doing an equality check, we don't explicitly specify a predicate
    Query::Ptr q1 = coll.createQuery();
    q1->appendLT("name", 20000);
    vector<PoseMetaPtr> res = coll.queryList(q1, false);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return size: %d", res.size());
    for (int i = 0; i < res.size(); i++)
    {
      std::cout 
          << "< meta: " << res[i]->lookupInt("name") << " >---"
          << "< tag_id: " << res[i]->tag_id << " >---"
          << "< p_x: " << res[i]->pose.position.x << " >---"
          << "< p_y: " << res[i]->pose.position.y<< " >---"
          << "< p_z: " << res[i]->pose.position.z
          << std::endl;
    }

  }

  // void update_db(std::shared_ptr<rock_rhino_msgs::msg::DetectedTag> msg)
  // {

  //   //check if the tag has been existed in db by search tag id
  //   Query::Ptr q1 = coll.createQuery();
  //   int int_id = msg->tag_id;
  //   q1->append("ID", int_id);
  //   vector<PoseMetaPtr> res = coll.queryList(q1, true);
  //   if (res.empty()) {
  //     RCLCPP_INFO(this->get_logger(), "add a new tag");
  //     const gm::msg::Pose p1 = makePose(msg);
  //     coll.insert(p1, makeMetadata(coll, msg->tag_id));
  //     update_webot_ui();

  //   }

  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return: %d", res[0]->lookupInt("ID"));
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "return: %d", res.empty());

  // }

  void update_webot_ui()
  {
     RCLCPP_INFO(this->get_logger(), "update_webots_ui");
     //send request to tcpip server to add new points
  }
};  // class ProCtrl2DataBaseActionServer

}  // namespace db_test

RCLCPP_COMPONENTS_REGISTER_NODE(db_test::ProCtrl2DataBaseActionServer)
