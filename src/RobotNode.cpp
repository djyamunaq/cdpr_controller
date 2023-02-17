#include <memory>
#include <chrono>
#include <iostream>
#include "TrajectoryGenerator.h"
#include "RobotModel.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"

#define MAX_DIST 100

using namespace std::chrono_literals;
using std::cout;
using std::cin;
using std::endl;
using std::placeholders::_1;

Eigen::Vector3d Xd;
bool moving = false;
double x, y, xPrev, yPrev;
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
  Eigen::Vector3d pis = {xPrev, yPrev, 1}; 
  Eigen::Vector3d pfs = {x, y, 1}; 

  // Time range and initial time of movement
  double ti = 0;
  Dt = 10*(pfs - pis).norm()/MAX_DIST;
  trajectory = TrajectoryGenerator(pis, pfs, Dt, ti);
}
  
class RobotNode : public rclcpp::Node {
  public:
    RobotNode() : Node("robot_node") {
      this->declare_parameter("nodeId", 1);
      this->nodeId = this->get_parameter("nodeId").get_parameter_value().get<unsigned int>();
      this->subPosition = this->create_subscription<geometry_msgs::msg::Point>("desired_pos", 10, std::bind(&RobotNode::topic_callback, this, _1));
      this->leftMotorPub = this->create_publisher<std_msgs::msg::Int32>("motor_left_pos", 10);
      this->rightMotorPub = this->create_publisher<std_msgs::msg::Int32>("motor_right_pos", 10);
      this->timer_ = this->create_wall_timer(10ms, std::bind(&RobotNode::timer_callback, this));
      this->rm = RobotModel(Eigen::Vector3d(48.50, 18.50, 1.0), Eigen::Vector3d(3.243, 90.752, 1.0),  Eigen::Vector3d(93.745, 90.752, 1.0), 0.2);
      dt = 1e-2;
      x = 48.50;
      y = 18.50;
    }

  private:
    void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) const {
      if(msg->x != x || msg->y != y) {
        xPrev = x;
        yPrev = y;
        x = msg->x;
        y = msg->y;
        
        generateTrajectory();

        moving = true;
      }
    }

    void timer_callback() {
      if(moving) {
        cout << "MOVING" << endl;

        Xd = trajectory.X(t);

        /* IK */
        int qL, qR;
        cout << "Xd: " << Xd.transpose() << endl;
        cout << "qL: " << qL << "| qR: " << qR << endl;

        rm.inverseKinematics(Xd, qL, qR);
        /* END IK */

        t += dt;
        if(t >= Dt) {
          moving = false;
          t = 0;
          cout << "END MOVE" << endl;
        }

        auto message = std_msgs::msg::Int32();
        message.data = qL;
        leftMotorPub->publish(message);
        message.data = qR;
        rightMotorPub->publish(message);
      }
    }

    unsigned int nodeId;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subPosition;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leftMotorPub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rightMotorPub;
    size_t count_;
    RobotModel rm;
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
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
  return 0;
}
