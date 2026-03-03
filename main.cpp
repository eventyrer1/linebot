#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : Node("keyboard_teleop")
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    if (!configure_terminal()) {
      RCLCPP_ERROR(get_logger(), "Failed to configure terminal for teleop input");
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { this->poll_keyboard(); });

    RCLCPP_INFO(get_logger(), "Keyboard teleop active. Arrows/WASD to move, SPACE to stop, q to quit.");
  }

  ~KeyboardTeleopNode() override
  {
    restore_terminal();
    publish_stop();
  }

private:
  bool configure_terminal()
  {
    if (tcgetattr(STDIN_FILENO, &old_term_) != 0) {
      return false;
    }

    termios raw = old_term_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      return false;
    }

    int current_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (current_flags == -1) {
      return false;
    }

    old_flags_ = current_flags;
    if (fcntl(STDIN_FILENO, F_SETFL, current_flags | O_NONBLOCK) == -1) {
      return false;
    }

    terminal_configured_ = true;
    return true;
  }

  void restore_terminal()
  {
    if (!terminal_configured_) {
      return;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    terminal_configured_ = false;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  void publish_cmd(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);
  }

  void poll_keyboard()
  {
    char c;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == 'q' || c == 'Q') {
        publish_stop();
        rclcpp::shutdown();
        return;
      }

      if (c == ' ' || c == 's' || c == 'S') {
        publish_stop();
        continue;
      }

      if (c == 'w' || c == 'W') {
        publish_cmd(0.5, 0.0);
        continue;
      }

      if (c == 'x' || c == 'X') {
        publish_cmd(-0.5, 0.0);
        continue;
      }

      if (c == 'a' || c == 'A') {
        publish_cmd(0.0, 0.7);
        continue;
      }

      if (c == 'd' || c == 'D') {
        publish_cmd(0.0, -0.7);
        continue;
      }

      if (c == '\x1b') {
        char seq1 = 0;
        char seq2 = 0;
        if (read(STDIN_FILENO, &seq1, 1) > 0 && read(STDIN_FILENO, &seq2, 1) > 0 && seq1 == '[') {
          if (seq2 == 'A') {
            publish_cmd(0.5, 0.0);
          } else if (seq2 == 'B') {
            publish_cmd(-0.5, 0.0);
          } else if (seq2 == 'C') {
            publish_cmd(0.0, -0.7);
          } else if (seq2 == 'D') {
            publish_cmd(0.0, 0.7);
          }
        }
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  termios old_term_{};
  int old_flags_ = 0;
  bool terminal_configured_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
  return 0;
}