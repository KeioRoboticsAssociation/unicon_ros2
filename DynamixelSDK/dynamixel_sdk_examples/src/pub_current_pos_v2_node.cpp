// Copyright (c) 2023 Takumi Asada
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

/******************************************************************************/
/* include                                                                    */
/******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "pub_dynamixel_data_node.hpp"

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
JointPubNode::JointPubNode()
: Node("pub_dynamixel_data_node")
{
  publisher_ = create_publisher<SetPosition>("/set_position", 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&JointPubNode::publishData, this)
  );
}

/******************************************************************************/
/* Function                                                                   */
/******************************************************************************/
void JointPubNode::publishData()
{
  SetPosition msg;
  
  // position range: 0 - 4095 
  msg.position = positions_5_[current_position_index_];
  msg.id = 1;    

  RCLCPP_INFO(get_logger(), "Publishing ID: %d Position: %d", msg.id, msg.position);

  publisher_->publish(msg);
  current_position_index_ = (current_position_index_ + 1) % positions_5_.size();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}