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

#ifndef BT_NAV_WP__GETWAYPOINT_HPP_
#define BT_NAV_WP__GETWAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav_wp
{

// Clase GetWaypoint  que será un nodo de acción en el behavior tree
// Se encarga de obtener un waypoint (punto de paso) a partir de coordenadas x, y y yaw
class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  // Definimos los puertos del nodo (los que usa el nodo para comunicarse con otros nodos)
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        // Puertos de entrada
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("yaw"),

        // Puerto de salida
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
      });
  }

private:
  // Variable para almacenar el waypoint
  geometry_msgs::msg::PoseStamped wp_;

  // Variables para almacenar las coordenadas x, y y la orientación yaw
  double x_, y_, yaw_;
};

}
#endif
