// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "rock_rhino_msgs/msg/detected_tag.hpp"
#include "rock_rhino_tcpip_service/visibility_control.h"

namespace rock_rhino_tcpip_service
{
class ListenerBestEffort : public rclcpp::Node
{
public:
  ROCK_RHINO_TCPIP_SERVICE_PUBLIC
  explicit ListenerBestEffort(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](const typename rock_rhino_msgs::msg::DetectedTag::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: ");
      };

    sub_ = create_subscription<rock_rhino_msgs::msg::DetectedTag>("cmd_vel", rclcpp::SensorDataQoS(), callback);
  }

private:
  rclcpp::Subscription<rock_rhino_msgs::msg::DetectedTag>::SharedPtr sub_;
};

}  // namespace rock_rhino_tcpip_service

RCLCPP_COMPONENTS_REGISTER_NODE(rock_rhino_tcpip_service::ListenerBestEffort)
