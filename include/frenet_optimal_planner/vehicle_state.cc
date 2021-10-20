/** vehicle_state.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Date types and functions related to vehicle state
 */

#include "vehicle_state.h"

namespace fop
{

VehicleState::VehicleState(const double x, const double y, const double yaw, const double speed)
{
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->v = speed;
}

ActuatorState::ActuatorState(const double steering_angle, const double acceleration)
{
  this->delta = steering_angle;
  this->a = acceleration;
}

}  // namespace fop
