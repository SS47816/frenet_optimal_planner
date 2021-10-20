/** math_utils.h
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Commonly used math functions
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <vector>
#include <cmath>

namespace frenet_optimal_planner
{

// Return PI
constexpr double pi();

// Convert degrees to radians
double deg2rad(const double x);

// Convert radians to degrees
double rad2deg(const double x);

// Convert metre per second to kilometers per hour
double mpsTokph(const double x);

// Convert kilometers per hour to metre per second
double kphTomps(const double x);

// Convert angle into range [-pi, +pi]
double unifyAngleRange(const double angle);

// Limit the value within [lower_bound, upper_bound]
double limitWithinRange(double value, const double lower_bound, const double upper_bound);

// Calculate the Euclideam distance between two points
double distance(const double x1, const double y1, const double x2, const double y2);

// Return true if a is greater or equal to b
bool cmp_doubles_greater_or_equal(const double a, const double b);

} // end of namespace frenet_optimal_planner

#endif // MATH_UTILS_H_