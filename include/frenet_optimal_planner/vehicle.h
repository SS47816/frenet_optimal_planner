#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "math_utils.h"
#include "Eigen/Core"

namespace fop
{
enum LINK_TYPE
{
  BASE_LINK,
  FRONT_LINK
};

class Vehicle
{
 public:
  // double max_speed, max_acceleration, max_deceleration;
  // double max_steering_angle, max_curvature, steering_angle_rate;
  // double L, Lf, Lr;  // distance between front and rear axles, distance from CoG to front/rear axle
  // Eigen::Vector2f min_point, max_point;

  static double length();
  static double width();
  static double L();
  static double Lf();
  static double Lr();

  static double max_steering_angle();
  static double max_speed();
  static double max_acceleration();
  static double max_deceleration();
  static double max_curvature(const double delta_t);
  static double steering_angle_rate();

};

}  // namespace fop

#endif  // VEHICLE_H_