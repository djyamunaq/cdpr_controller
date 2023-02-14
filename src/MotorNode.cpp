#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include <bits/stdc++.h>
#include "MotorController.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::vector;

epos::MotorController mc; 

void initNode(unsigned int nodeId) {
  mc.clearFault(nodeId);
  mc.activatePositionMode(nodeId);
  mc.setEnable(nodeId);
  mc.setMaxAcc(nodeId, 500);
}

class MotorNode : public rclcpp::Node {
  public:
    MotorNode() : Node("motor_node") {
      motorLeftSub = this->create_subscription<std_msgs::msg::Int32>("motor_left_pos", 10, std::bind(&MotorNode::leftMotorCallback, this, _1));
      motorRightSub = this->create_subscription<std_msgs::msg::Int32>("motor_right_pos", 10, std::bind(&MotorNode::rightMotorCallback, this, _1));

      readyFlagPub = this->create_publisher<std_msgs::msg::Bool>("ready_flag", 10);
      initNode(this->motorLeftId);
      initNode(this->motorRightId);
    }

  private:
    void leftMotorCallback(const std_msgs::msg::Int32::SharedPtr msg) const {
      // unsigned int nodeId = std::stoi(msgData.substr(0, msgData.find(delimiter)));

      int q = msg->data;
      cout << "qL: " << q << endl;

      mc.setPos(this->motorLeftId, -q);
    }
    void rightMotorCallback(const std_msgs::msg::Int32::SharedPtr msg) const {
      // unsigned int nodeId = std::stoi(msgData.substr(0, msgData.find(delimiter)));

      int q = msg->data;
      cout << "qR: " << q << endl;

      mc.setPos(this->motorRightId, q);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motorLeftSub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motorRightSub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readyFlagPub;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int motorRightId = 2;
    unsigned int motorLeftId = 1;
};

int main(int argc, char * argv[]) {
  /* =================================================================== */
  /* Motor configuration*/
  string deviceName = "EPOS2";
  string protocolName = MAXON_SERIAL_V2;
  string interfaceName = "USB";
  string portName = "USB0";
  unsigned int baudrate = 1e6; 

  mc.connect(deviceName, protocolName, interfaceName, portName, baudrate);
    
  /* =================================================================== */
  /* Spin subscriber */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
