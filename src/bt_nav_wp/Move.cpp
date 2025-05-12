// Copyright 2019 Intelligent Robotics Lab
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


#include "bt_nav_wp/Move.hpp"

namespace bt_nav_wp  
{
  
// Constructor de la clase Move, inicializamos el nodo de acción
Move::Move(

  // Nombre del nodo en el behavior tree
  const std::string & xml_tag_name,
  
  // Nombre de la acción (en este caso "navigate_to_pose")
  const std::string & action_name,

  // Configuración del nodo en el behavior tree que hereda de BtActionNode para interactuar con el Action Server
  const BT::NodeConfiguration & conf)
  : bt_nav_wp::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)  
{
}


/* Método que se llama en cada ciclo del behavior tree, se encarga de obtener la posición y orientación
del objetivo (goal) desde el behavior tree*/
void
Move::on_tick()
{
  // Variable para almacenar la posición y orientación del objetivo
  geometry_msgs::msg::PoseStamped goal;

  // Obtenemos el objetivo (goal) del behavior tree
  getInput("goal", goal);

  // Imprimimos las coordenadas x, y del objetivo (más claridad)
  RCLCPP_INFO(node_->get_logger(), "Move: %.2f %.2f", goal.pose.position.x, goal.pose.position.y);

  // Guardamos la posicion del objetivo en la variable goal_
  goal_.pose = goal;               
  
  // Establecemos que las coordenadas están en el sistema de referencia "map"
  goal.header.frame_id = "map";               
}


// Método que se llama cuando la acción se completa con éxito
BT::NodeStatus
Move::on_success()
{
  // Imprimimos un mensaje indicando que la navegación fue exitosa
  RCLCPP_INFO(node_->get_logger(), "** NAVIGATION SUCCEEDED **");

  // Devolvemos el estado de éxito al behavior tree
  return BT::NodeStatus::SUCCESS;  
}


// Registramos el nodo en el sistema de Behavior Tree para poder usarlo dentro de BTs
BT_REGISTER_NODES(factory)
{
  // Definimos cómo construir el nodo Move
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // Creamos una nueva instancia del nodo Move, pasando el nombre y configuración al constructor
      return std::make_unique<bt_nav_wp::Move>(name, "navigate_to_pose", config);         
    };

  // Registramos el nodo Move en la "fábrica" de nodos de behavior tree
  factory.registerBuilder<bt_nav_wp::Move>("Move", builder);
}

}
