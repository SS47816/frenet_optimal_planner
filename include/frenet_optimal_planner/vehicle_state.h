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

namespace frenet_optimal_planner
{


class VehicleState
{
 public:
  // Constructors
  VehicleState(){};
  VehicleState(const double x, const double y, const double yaw, const double speed);
  // Destructor
  virtual ~VehicleState(){};

  double x;
  double y;
  double yaw;  // yaw
  double v;    // velocity
};

class ActuatorState
{
 public:
  // Constructors
  ActuatorState(){};
  ActuatorState(const double steering_angle, const double acceleration);
  // Destructor
  virtual ~ActuatorState(){};

  double delta;  // steering angle
  double a;      // acceleration
};

}  // namespace frenet_optimal_planner

#endif  // VEHICLE_STATE_H_