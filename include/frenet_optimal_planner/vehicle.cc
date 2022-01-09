#include "vehicle.h"

namespace fop
{
  double Vehicle::length() { return 5.0; };
  double Vehicle::width() { return 2.0; };
  double Vehicle::L() { return 2.75; };
  double Vehicle::Lf() { return 1.25; };
  double Vehicle::Lr() { return 1.5; };

  double Vehicle::max_steering_angle() { return deg2rad(35); };
  double Vehicle::max_speed() { return kph2mps(100); };               // TODO: change back to 3.33
  double Vehicle::max_acceleration() { return kph2mps(100)/15.0; };
  double Vehicle::max_deceleration() { return -kph2mps(100)/5.0; };
  double Vehicle::max_curvature() { return 1.0; };                    // TODO: origin 1.0  0.476 (for 30 degree), 
  double Vehicle::steering_angle_rate() { return deg2rad(35)/3.0; }; 

} // namespace fop