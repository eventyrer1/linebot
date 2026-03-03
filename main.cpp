#include <rclcpp/rclcpp.hpp>                 // ROS 2 C++ client library
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>                         // Terminal raw mode (Linux)
#include <unistd.h>                          // read(), STDIN_FILENO


// Very small keyboard teleop:
// - Arrow Up    -> forward
// - Arrow Down  -> backward
// - Arrow Left  -> turn left
// - Arrow Right -> turn right
// - q           -> quit
int main(int argc, char **argv)
{
  // Initialize ROS 2 runtime
  rclcpp::init(argc, argv);

  // Create a simple node named "keyboard_drive"
  auto node = rclcpp::Node::make_shared("keyboard_drive");

  // Create publisher to /cmd_vel
  // Queue size 10 is enough for this simple use-case
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Save terminal settings so we can restore them on exit
  termios oldt{}, newt{};
  tcgetattr(STDIN_FILENO, &oldt);   // read current terminal config
  newt = oldt;                      // start from current config

  // Disable canonical mode + echo:
  // - ICANON off => key presses available immediately (no Enter needed)
  // - ECHO off   => keys are not printed to terminal
  newt.c_lflag &= ~(ICANON | ECHO);

  // Apply new terminal settings now
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // Main key-reading loop
  while (rclcpp::ok()) {
    char c = 0;

    // Read exactly one byte from keyboard
    // If read fails or returns nothing, continue
    if (read(STDIN_FILENO, &c, 1) <= 0) continue;

    // Default-constructed Twist = all zeros = stop
    geometry_msgs::msg::Twist msg;

    // Quit if user presses q
    if (c == 'q') break;

    // Arrow keys arrive as 3-byte escape sequence:
    // ESC (0x1b), '[' , one of A/B/C/D
    if (c == '\x1b') {
      char a = 0, b = 0;

      // Read next two bytes in the escape sequence
      if (read(STDIN_FILENO, &a, 1) <= 0) continue;
      if (read(STDIN_FILENO, &b, 1) <= 0) continue;

      // Confirm this is a CSI sequence: ESC [
      if (a == '[') {
        // Map arrow key to velocity commands
        if (b == 'A') msg.linear.x = 0.3;    // Up    -> forward
        if (b == 'B') msg.linear.x = -0.3;   // Down  -> backward
        if (b == 'C') msg.angular.z = -1.0;  // Right -> rotate right
        if (b == 'D') msg.angular.z = 1.0;   // Left  -> rotate left
      }
    }

    // Publish current command
    // If key was not an arrow, msg stays zero (stop)
    pub->publish(msg);
  }

  // Publish one final zero command so robot stops when quitting
  pub->publish(geometry_msgs::msg::Twist());

  // Restore original terminal settings (important)
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  // Shutdown ROS 2 cleanly
  rclcpp::shutdown();
  return 0;
}