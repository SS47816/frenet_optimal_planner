#include "vehicle.h"

namespace fop
{
  double Vehicle::length() { return 5.0; };
  double Vehicle::width() { return 2.0; };
  double Vehicle::L() { return 2.75; };
  double Vehicle::Lf() { return 1.25; };
  double Vehicle::Lr() { return 1.5; };

  double Vehicle::max_steering_angle() { return deg2rad(35); };
  double Vehicle::max_speed() { return kph2mps(100); };
  double Vehicle::max_acceleration() { return max_speed()/10.0; };
  double Vehicle::max_deceleration() { return -max_speed()/5.0; };
  double Vehicle::max_curvature(const double delta_t) { return steering_angle_rate()/Lr()*delta_t; };
  double Vehicle::steering_angle_rate() { return max_steering_angle()/3.0; }; 

} // namespace fop