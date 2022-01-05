/** vehicle_state.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Date types and functions related to vehicle state
 */

#include "vehicle_state.h"

namespace fop
{

VehicleState::VehicleState(const float x, const float y, const float yaw, const float speed)
  : x(x), y(y), yaw(yaw), v(0) {};

ActuatorState::ActuatorState(const float steering_angle, const float acceleration)
  : delta(steering_angle), a(acceleration) {};

}  // namespace fop
