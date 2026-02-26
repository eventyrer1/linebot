#pragma once

#include <geometry_msgs/msg/twist.hpp>

namespace linebot
{

class Rectangle
{
public:
  Rectangle(double v,
            double long_side,
            double short_side,
            double w);

  geometry_msgs::msg::Twist compute(double t);

private:
  double v_;
  double long_side_;
  double short_side_;
  double w_;
};

}  // namespace linebot