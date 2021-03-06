/** quartic_polynomial.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Quartic Polynomial
 */

#include "quartic_polynomial.h"

namespace fop
{

QuarticPolynomial::QuarticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T)
{
  Eigen::MatrixXd A = Eigen::MatrixXd(2, 2);
  A << 3 * T * T, 4 * T * T * T,
      6 * T, 12 * T * T;

  Eigen::MatrixXd B = Eigen::MatrixXd(2, 1);
  B << end[0] - (start[1] + start[2] * T),
      end[1] - start[2];

  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai * B;

  coefficients = {start[0], start[1], .5 * start[2]};

  for (int i = 0; i < C.size(); i++)
  {
    coefficients.push_back(C.data()[i]);
  }
};

// calculate the s/d coordinate of a point
double QuarticPolynomial::calculatePoint(double t)
{
  return coefficients[0] + coefficients[1]*t + coefficients[2]*t*t + coefficients[3]*t*t*t + coefficients[4]*t*t*t*t;
}

double QuarticPolynomial::calculateFirstDerivative(double t)
{
  return coefficients[1] + 2*coefficients[2]*t + 3*coefficients[3]*t*t + 4*coefficients[4]*t*t*t;
}

double QuarticPolynomial::calculateSecondDerivative(double t)
{
  return 2*coefficients[2]*t + 6*coefficients[3]*t + 12*coefficients[4]*t*t;
}

double QuarticPolynomial::calculateThirdDerivative(double t)
{
  return 6*coefficients[3] + 24*coefficients[4]*t;
}

} // namespace fop
