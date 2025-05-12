// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>

#include "bt_nav_wp/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bt_nav_wp
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  getInput<double>("x", x_);
  getInput<double>("y", y_);
  getInput<double>("yaw", yaw_);

  wp_.header.frame_id = "map";
  wp_.pose.orientation.w = 1.0;

  wp_.pose.position.x = x_;
  wp_.pose.position.y = y_;
  wp_.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  wp_.pose.orientation.x = q.x();
  wp_.pose.orientation.y = q.y();
  wp_.pose.orientation.z = q.z();
  wp_.pose.orientation.w = q.w();
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  setOutput("waypoint", wp_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav_wp

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav_wp::GetWaypoint>("GetWaypoint");
}
