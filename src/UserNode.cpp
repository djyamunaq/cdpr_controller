#include <chrono>
#include <memory>
#include <future>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;
using std::cout;
using std::cin;
using std::endl;

unsigned int x, y, xPrev, yPrev;

int getData() {
  cin >> x >> y;

  cout << "X: " << x << " | " << "Y: " << endl;
  return 0;
}

class UserNode : public rclcpp::Node {
  public:
    UserNode() : Node("user_node"), count_(0) {
      positionPub = this->create_publisher<geometry_msgs::msg::Point>("desired_pos", 10);
      
      // this->create_subscription<std_msgs::msg::Bool>('ready_flag', 10, std::bind(&UserNode::readyFlagCallback, this, std::placeholders::_1))
      timer_ = this->create_wall_timer(10ms, std::bind(&UserNode::timer_callback, this));
    }

  private:
    void readyFlagCallback() {
      cout << "Ready" << endl;
    }

    void timer_callback() {
      auto message = geometry_msgs::msg::Point ();

      std::future<int> futureCounter  = std::async(std::launch::async, getData);
      std::future_status status = futureCounter.wait_until(std::chrono::system_clock::now() + std::chrono::nanoseconds(100));

      cout << "LOOP" << endl;
      // if(xPrev != x || yPrev != y) {
        // xPrev = x;
        // yPrev = y;
        // RCLCPP_INFO(this->get_logger(), "x: " + std::to_string(x) + " | y: " + std::to_string(y));
      // }
      
      // message. = std::to_string(x) + "|" + std::to_string(y);
      // positionPub->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr positionPub;
    size_t count_;
};

int main(int argc, char * argv[]) {

  /* =================================================================== */
  /* Spin publisher */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UserNode>());
  rclcpp::shutdown();

  return 0;
}
