// main.cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rectangle.hpp"

using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("driver"),
    rectangle_(0.25, 4.0, 2.0, 0.6)   // v, long_side, short_side, turn_rate
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    start_ = this->now();
    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&DriverNode::tick, this)
    );
  }

private:
  void tick()
  {
    double t = (this->now() - start_).seconds();
    auto cmd = rectangle_.compute(t);
    pub_->publish(cmd);
  }

  linebot::Rectangle rectangle_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverNode>());
  rclcpp::shutdown();
  return 0;
}