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

// Constructor de la clase GetWaypoint
GetWaypoint::GetWaypoint(

  // Nombre del nodo en el behavior tree 
  const std::string & xml_tag_name,
  
  // Configuración del nodo, que hereda de ActionNodeBase (para interactuar con el Action Server)
  const BT::NodeConfiguration & conf)  
: BT::ActionNodeBase(xml_tag_name, conf)

{
  // Creamos un puntero a nodo compartido
  rclcpp::Node::SharedPtr node;
  
  // Obtenemos el nodo de configuración desde el Blackboard
  // Blackboard --> espacio de memoria compartido entre los nodos
  config().blackboard->get("node", node); 

  // Obtenemos valores "x" "y" "yaw" del behavior tree y los guardamos en x_ y_ y yaw_
  getInput<double>("x", x_);  
  getInput<double>("y", y_);  
  getInput<double>("yaw", yaw_);

  // Establecemos que las coordenadas del wp están en el sistema de referencia "map"
  wp_.header.frame_id = "map";
  
  // Inicializamos la orientación con un cuaternión de valor 1.0
  wp_.pose.orientation.w = 1.0;  

  // Asignamos las coordenadas del waypoint (x, y) y la orientación (yaw)
  wp_.pose.position.x = x_;  
  wp_.pose.position.y = y_; 
  wp_.pose.position.z = 0.0; 

  // Creamos un objeto de tipo cuaternión para la orientación
  tf2::Quaternion q;

  // Establecemos el cuaternión usando los valores de Roll, Pitch y Yaw
  q.setRPY(0.0, 0.0, yaw_);

  // Asignamos las componentes del cuaternion
  wp_.pose.orientation.x = q.x();  
  wp_.pose.orientation.y = q.y();  
  wp_.pose.orientation.z = q.z();  
  wp_.pose.orientation.w = q.w();


// Generalmente se usa este metodo para detener o limpiar la acción
void
GetWaypoint::halt()
{
}

// Método que se llama en cada ciclo del behavior tree para que el nodo realice su tarea
BT::NodeStatus
GetWaypoint::tick()
{
  // Establecemos el waypoint como salida del nodo
  setOutput("waypoint", wp_);
  
  // El nodo ha tenido éxito (se ha llegado al waypoint)
  return BT::NodeStatus::SUCCESS;  
}


}

#include "behaviortree_cpp_v3/bt_factory.h"

// Registramos el nodo en el sistema de Behavior Tree para poder usarlo dentro de BTs
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav_wp::GetWaypoint>("GetWaypoint");
}
