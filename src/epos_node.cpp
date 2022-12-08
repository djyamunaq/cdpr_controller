// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include <bits/stdc++.h>
#include "MotorController.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::vector;

epos::MotorController mc; 
vector<int> knownNodes;

void initNode(unsigned int nodeId) {
  mc.clearFault(nodeId);
  mc.activatePositionMode(nodeId);
  mc.setEnable(nodeId);
  mc.setMaxAcc(nodeId, 500);
}

class EposNode : public rclcpp::Node {
  public:
    EposNode() : Node("epos_master") {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "motor_pos", 10, std::bind(&EposNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      std::string msgData = msg->data.c_str();
      std::string delimiter = "|";
      unsigned int nodeId = std::stoi(msgData.substr(0, msgData.find(delimiter)));
      double pos = std::stod(msgData.substr(msgData.find(delimiter)+1, msgData.length()));

      if(std::find(knownNodes.begin(), knownNodes.end(), nodeId) == knownNodes.end()) {
        knownNodes.push_back(nodeId);
        sort(knownNodes.begin(), knownNodes.end());
        initNode(nodeId);
        RCLCPP_INFO(this->get_logger(), "Epos Node: Node " + std::to_string(nodeId) + " initialized");
      }

      RCLCPP_INFO(this->get_logger(), "Epos Node " + std::to_string(nodeId) + " pos: " + std::to_string(pos));
      mc.setPos(nodeId, pos);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  /* =================================================================== */
  /* Motor configuration*/
  unsigned int errorCode;
  string deviceName = "EPOS2";
  string protocolName = MAXON_SERIAL_V2;
  string interfaceName = "USB";
  string portName = "USB0";
  unsigned int baudrate = 1e6; 

  mc.connect(deviceName, protocolName, interfaceName, portName, baudrate);
    
  /* =================================================================== */
  /* Spin subscriber */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EposNode>());
  rclcpp::shutdown();
  return 0;
}
