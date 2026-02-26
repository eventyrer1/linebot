// rectangle.cpp
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

// Returns a cmd_vel that drives a rectangle.
// t: seconds since start (monotonic)
// v: forward speed (m/s)
// long_side, short_side: meters
// w: turn rate (rad/s)
geometry_msgs::msg::Twist rectangle_cmd(
  double t,
  double v = 0.25,
  double long_side = 4.0,
  double short_side = 2.0,
  double w = 0.6)
{
  const double t_long  = long_side / v;
  const double t_short = short_side / v;
  const double t_turn  = (M_PI / 2.0) / w;  // 90 degrees

  const double cycle = 2.0 * (t_long + t_short) + 4.0 * t_turn;
  double p = std::fmod(t, cycle);

  geometry_msgs::msg::Twist cmd;

  auto forward = [&](double dur) -> bool {
    if (p < dur) { cmd.linear.x = v; cmd.angular.z = 0.0; return true; }
    p -= dur; return false;
  };
  auto turn = [&](double dur) -> bool {
    if (p < dur) { cmd.linear.x = 0.0; cmd.angular.z = w; return true; }
    p -= dur; return false;
  };

  // long -> turn -> short -> turn -> long -> turn -> short -> turn
  forward(t_long)  || turn(t_turn) ||
  forward(t_short) || turn(t_turn) ||
  forward(t_long)  || turn(t_turn) ||
  forward(t_short) || turn(t_turn);

  return cmd;
}