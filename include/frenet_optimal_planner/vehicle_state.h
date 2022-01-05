/** vehicle_state.h
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * VehicleState class
 * ActuatorState class
 */

#ifndef VEHICLE_STATE_H_
#define VEHICLE_STATE_H_

#include <cmath>

namespace fop
{


struct VehicleState
{
 public:
  float x;
  float y;
  float yaw;  // yaw
  float v;    // velocity

  // Constructors
  VehicleState() {};
  VehicleState(const float x, const float y, const float yaw, const float speed)
    : x(x), y(y), yaw(yaw), v(0) {};
};

struct ActuatorState
{
 public:
  float delta;  // steering angle
  float a;      // acceleration

  // Constructors
  ActuatorState() {};
  ActuatorState(const float steering_angle, const float acceleration)
    : delta(steering_angle), a(acceleration) {};
};

}  // namespace fop

#endif  // VEHICLE_STATE_H_