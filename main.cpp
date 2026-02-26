#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class SimpleDriver : public rclcpp::Node
{
public:
  SimpleDriver() : Node("simple_driver")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    start_ = this->now();
    timer_ = this->create_wall_timer(50ms, std::bind(&SimpleDriver::tick, this));
  }

private:
  void tick()
  {
    const auto t = (this->now() - start_).seconds();

    geometry_msgs::msg::Twist cmd;

    // 0-2s: forward, 2-3s: rotate, then repeat every 3s
    const double phase = std::fmod(t, 3.0);
    if (phase < 2.0) {
      cmd.linear.x = 0.2;   // m/s
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 1.0;  // rad/s
    }

    pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleDriver>());
  rclcpp::shutdown();
  return 0;
}