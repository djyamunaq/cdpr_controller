#include <memory>
#include <chrono>
#include <iostream>
#include "TrajectoryGenerator.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::cout;
using std::cin;
using std::endl;
using std::placeholders::_1;

bool moving = false;
unsigned int x, y, xPrev, yPrev;
double dt, Dt, t;
TrajectoryGenerator trajectory;

double motor_theta;
double motor_radius;
double pulley_r;
Eigen::Vector2d pulley_pos;
double wire_r;
double ee_s;
Eigen::Vector3d ee_pos;
double ee_theta;

void generateTrajectory() {
  // Initial and final positions of End-Effector frame origin
  Eigen::Vector2d pis = {xPrev, yPrev}; 
  Eigen::Vector2d pfs = {x, y}; 

      // Time range and initial time of movement
  double ti = 1;

  trajectory = TrajectoryGenerator(pis, pfs, Dt, ti);
}

class MotorNode : public rclcpp::Node {
  public:
    MotorNode() : Node("robot_motor") {
      this->declare_parameter("nodeId", 1);
      // this->nodeId = this->get_parameter("nodeId").get_parameter_value().get<unsigned int>();

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "robot_pos", 10, std::bind(&MotorNode::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("motor_pos", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&MotorNode::timer_callback, this));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      std::string msgData = msg->data.c_str();
      std::string delimiter = "|";
      x = std::stoi(msgData.substr(0, msgData.find(delimiter)));
      y = std::stoi(msgData.substr(msgData.find(delimiter)+1, msgData.length()));
      
      if((x != xPrev || y != yPrev) && !moving) {
        generateTrajectory();
        moving = true;
      }

      RCLCPP_INFO(this->get_logger(), "Motor Node " + std::to_string(this->get_parameter("nodeId").get_parameter_value().get<unsigned int>()) + ": " + std::to_string(x) + "|" + std::to_string(y));
    }

    void timer_callback() {
      if(moving) {
          // Current position
        Eigen::Vector2d X = trajectory.X(t);
          // Desired position
        // Eigen::Vector2d Xd = trajectory.X(t+dt);
            
          // Transformation Matrix (EE Local frame to World Frame) node 1
        Eigen::Matrix<double, 3, 3> T;
        T << cos(ee_theta), -sin(ee_theta), X[0],
             sin(ee_theta), cos(ee_theta), X[1],
             0, 0, 1;
        // Eigen::Matrix<double, 3, 3> Td;
        // Td << cos(ee_theta), -sin(ee_theta), Xd[0],
        //      sin(ee_theta), cos(ee_theta), Xd[1],
        //      0, 0, 1;

            // Calculate positions in world frame 
        Eigen::Vector3d X_world = T*ee_pos;
        // Eigen::Vector3d Xd_world = Td*ee_pos;

        double wire_len = (pulley_pos - X_world.block<2, 1>(0, 0)).norm();
        // double wire_len_d = (pulley_pos - Xd_world.block<2, 1>(0, 0)).norm();

            // Calculate variation in wire length
        // double Dwire_len = wire_len_d - wire_len;

            // Calculate motor angular variation
        // double Dth = Dwire_len/((double) 2*M_PI*motor_radius);

        /* Wire in range [0, 200] */
        this->pos = 1000*wire_len;
        cout << this->pos << endl;

        t += dt;
        if(t >= Dt) moving = false;
      }

      auto message = std_msgs::msg::String();

      message.data = std::to_string(this->get_parameter("nodeId").get_parameter_value().get<unsigned int>()) + "|" + std::to_string(this->pos);
      publisher_->publish(message);
    }
    double pos;
    unsigned int nodeId;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
  /* =================================================================== */ 
  /* Trajectory time configuration */
  dt = 1e-3, Dt = 1, t=0;

  /* =================================================================== */ 
  /* Motor configuration */
      // Motor Initial Position in degrees
  motor_theta = 0;
      // Motor radius
  motor_radius = 1;

  /* =================================================================== */ 
  /* Pulley configuration */
      // Pulley radius
  pulley_r = 1;
      // Pulley position
  pulley_pos << 15, 15;
    
  /* =================================================================== */ 
  /* Wire configuration */
      // Wire radius
  wire_r = 1;

  /* =================================================================== */ 
  /* End-Effector configuration */
      // EE side size
  ee_s = 2;
      // EE top-left corner position
  ee_pos << -ee_s/2, ee_s/2, 1;
      // EE orientation
  ee_theta = 0;

  /* =================================================================== */
  /* Spin subscriber */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
