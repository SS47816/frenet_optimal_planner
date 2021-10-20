#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "math_utils.h"

namespace frenet_optimal_planner
{
enum LINK_TYPE
{
  BASE_LINK,
  FRONT_LINK
};

class Vehicle
{
public:
  static double length();
  static double width();
  static double L();
  static double Lf();
  static double Lr();

  static double max_steering_angle();
  static double max_speed();
  static double max_acceleration();
  static double max_deceleration();
  static double max_curvature();
  static double steering_angle_rate();

private:
};

}  // namespace frenet_optimal_planner

#endif  // VEHICLE_H_