
#include "linebot/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    tcgetattr(STDIN_FILENO, &original_);
    raw_ = original_;
    raw_.c_lflag &= ~(ICANON | ECHO);
    raw_.c_cc[VMIN] = 0;
    raw_.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_);

    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);
  }

  ~TerminalRawMode()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    fcntl(STDIN_FILENO, F_SETFL, original_flags_);
  }

private:
  termios original_{};
  termios raw_{};
  int original_flags_{0};
};

int run_controller(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyboard_teleop");
  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  TerminalRawMode terminal_mode;

  RCLCPP_INFO(node->get_logger(), "WASD teleop ready. Press keys directly (q to quit).");

  rclcpp::Rate loop_rate(50);
  while (rclcpp::ok()) {
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);

    if (n > 0) {
      geometry_msgs::msg::Twist cmd;

      if (c == 'w' || c == 'W') {
        cmd.linear.x = 0.5;
      } else if (c == 's' || c == 'S') {
        cmd.linear.x = -0.5;
      } else if (c == 'a' || c == 'A') {;
        cmd.angular.z = 3.0;
      } else if (c == 'd' || c == 'D') {
        cmd.angular.z = -3.0;
      } else if (c == 'q' || c == 'Q') {
        break;
      } else {
        continue;
      }

      cmd_pub->publish(cmd);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  cmd_pub->publish(geometry_msgs::msg::Twist());
  rclcpp::shutdown();
  return 0;
}
