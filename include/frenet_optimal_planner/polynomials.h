/** polynomials.h
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Helper functions related to polynomials
 */

#ifndef POLYNOMIALS_H_
#define POLYNOMIALS_H_

#include "Eigen/Dense"

using Eigen::VectorXd;

namespace frenet_optimal_planner
{

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x);
// Fit a polynomial.
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);

} // end of namespace frenet_optimal_planner

#endif // POLYNOMIALS_H_