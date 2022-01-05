/** vehicle_state.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Date types and functions related to vehicle state
 */

#include "vehicle_state.h"

namespace fop
{

VehicleState::VehicleState() {};

VehicleState::VehicleState(const double x, const double y, const double yaw, const double speed)
  : x(x), y(y), yaw(yaw), v(0) {};


ActuatorState::ActuatorState() {};

ActuatorState::ActuatorState(const double steering_angle, const double acceleration)
  : delta(steering_angle), a(acceleration) {};

}  // namespace fop
