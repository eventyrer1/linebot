#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyboard_teleop");
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(node->get_logger(), "WASD teleop ready. Type w/a/s/d then Enter.");

  char c;
  while (rclcpp::ok() && std::cin >> c) {
    geometry_msgs::msg::Twist cmd;

        if (c == 'w') {
          cmd.linear.x = 0.5;
        } else if (c == 's') {
          cmd.linear.x = -0.5;
        } else if (c == 'a') {
          cmd.linear.x = 0.15;
          cmd.angular.z = 1.2;
        } else if (c == 'd') {
          cmd.linear.x = 0.15;
          cmd.angular.z = -1.2;
        } else {
          continue;
        }
    

    cmd_pub->publish(cmd);
    rclcpp::spin_some(node);
  }

  cmd_pub->publish(geometry_msgs::msg::Twist());
  rclcpp::shutdown();
  return 0;
}