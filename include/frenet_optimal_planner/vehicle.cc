#include "vehicle.h"

namespace frenet_optimal_planner
{

  double Vehicle::length() { return 3.0; };
  double Vehicle::width() { return 1.4; };
  double Vehicle::L() { return 2.2; };
  double Vehicle::Lf() { return 1.1; };
  double Vehicle::Lr() { return 1.1; };

  double Vehicle::max_steering_angle() { return deg2rad(30); };
  double Vehicle::max_speed() { return 3.33; }; //TODO:change back to 3.33
  double Vehicle::max_acceleration() { return 1.0; };
  double Vehicle::max_deceleration() { return -1.0; };
  double Vehicle::max_curvature() { return 0.666; };//TODO: origin 1.0  0.476 (for 30 degree), 
  double Vehicle::steering_angle_rate() { return 0.144; }; 

} // namespace frenet_optimal_planner