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

#include <chrono>
#include <memory>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::cout;
using std::cin;
using std::endl;

unsigned int x, y, xPrev, yPrev;

int getData() {
  cin >> x >> y;
  return 0;
}

class MinimalPublisher : public rclcpp::Node {
  public:
    MinimalPublisher() : Node("robot_controller"), count_(0) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("robot_pos", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback() {
      auto message = std_msgs::msg::String();

      std::future<int> futureCounter  = std::async(std::launch::async, getData);
      std::future_status status = futureCounter.wait_until(std::chrono::system_clock::now() + std::chrono::nanoseconds(100));

      if(xPrev != x || yPrev != y) {
        xPrev = x;
        yPrev = y;
        RCLCPP_INFO(this->get_logger(), "x: " + std::to_string(x) + " | y: " + std::to_string(y));
      }
      
      message.data = std::to_string(x) + "|" + std::to_string(y);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {

  /* =================================================================== */
  /* Spin publisher */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}
