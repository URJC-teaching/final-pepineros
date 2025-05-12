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

#ifndef BT_NAV_WP__MOVE_HPP_
#define BT_NAV_WP__MOVE_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "bt_nav_wp/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_nav_wp
{

// Clase Move que se encarga de mover al Kobuki a una posición determinada
class Move : public bt_nav_wp::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  // Metodo que define los puertos del nodo (los que usa el nodo para comunicarse con otros nodos)
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        // Puerto de entrada (objetivo al que se quiere mover)
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
      });
  }

private:
};


}

#endif
