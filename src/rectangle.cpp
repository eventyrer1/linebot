#include "rectangle.hpp"
#include <cmath>

namespace linebot
{

Rectangle::Rectangle(double v, double long_side, double short_side, double w)
: v_(v), long_side_(long_side), short_side_(short_side), w_(w) {}

geometry_msgs::msg::Twist Rectangle::compute(double t)
{
  geometry_msgs::msg::Twist cmd;

  const double tL = long_side_ / v_;
  const double tS = short_side_ / v_;
  const double tT = (M_PI * 0.5) / w_;
  const double cycle = 2.0 * (tL + tS) + 4.0 * tT;

  const double p = std::fmod(t, cycle);

  // 8 phases: L, turn, S, turn, L, turn, S, turn
  const double start[8] = {0, tL, tL + tT, tL + tT + tS,
                           tL + 2*tT + tS, 2*tL + 2*tT + tS,
                           2*tL + 3*tT + tS, 2*tL + 3*tT + 2*tS};

  const bool is_forward =
      (p < start[1]) ||
      (p >= start[2] && p < start[3]) ||
      (p >= start[4] && p < start[5]) ||
      (p >= start[6] && p < start[7]);

  cmd.linear.x  = is_forward ? v_ : 0.0;
  cmd.angular.z = is_forward ? 0.0 : w_;

  return cmd;
}

} // namespace linebot