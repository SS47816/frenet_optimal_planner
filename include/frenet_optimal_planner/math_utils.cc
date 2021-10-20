/** math_utils.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Commonly used math functions
 */

#include "math_utils.h"

namespace frenet_optimal_planner
{

constexpr double pi() { return M_PI; }

double deg2rad(const double x) { return x * pi() / 180; }

double rad2deg(const double x) { return x * 180 / pi(); }

double mpsTokph(const double x) { return x * 3.6; }

double kphTomps(const double x) { return x / 3.6; }

double unifyAngleRange(const double angle)
{
  double new_angle = angle;
  while (new_angle > M_PI)
  {
    new_angle -= 2 * M_PI;
  }
  while (new_angle < -M_PI)
  {
    new_angle += 2 * M_PI;
  }
  return new_angle;
}

double limitWithinRange(double value, const double lower_bound, const double upper_bound)
{
  value = std::max(value, lower_bound);
  value = std::min(value, upper_bound);
  return value;
}

double distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool cmp_doubles_greater_or_equal(const double a, const double b)
{
  if ((a - b) >= 0.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

} // end of namespace frenet_optimal_planner