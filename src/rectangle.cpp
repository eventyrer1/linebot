#include "rectangle.hpp"
#include <cmath>

namespace linebot
{

Rectangle::Rectangle(double v,
                     double long_side,
                     double short_side,
                     double w)
: v_(v),
  long_side_(long_side),
  short_side_(short_side),
  w_(w)
{
}

geometry_msgs::msg::Twist Rectangle::compute(double t)
{
  geometry_msgs::msg::Twist cmd;

  const double t_long  = long_side_  / v_;
  const double t_short = short_side_ / v_;
  const double t_turn  = (M_PI / 2.0) / w_;

  const double cycle =
      2.0 * (t_long + t_short) +
      4.0 * t_turn;

  double p = std::fmod(t, cycle);

  auto forward = [&](double dur) -> bool {
    if (p < dur) {
      cmd.linear.x = v_;
      cmd.angular.z = 0.0;
      return true;
    }
    p -= dur;
    return false;
  };

  auto turn = [&](double dur) -> bool {
    if (p < dur) {
      cmd.linear.x = 0.0;
      cmd.angular.z = w_;
      return true;
    }
    p -= dur;
    return false;
  };

  forward(t_long)  || turn(t_turn) ||
  forward(t_short) || turn(t_turn) ||
  forward(t_long)  || turn(t_turn) ||
  forward(t_short) || turn(t_turn);

  return cmd;
}

}  // namespace linebot