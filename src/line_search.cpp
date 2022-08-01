#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double sigma = max_step;
  Eigen::VectorXd current = z - sigma * dz;
  proj_z(current);
  while (f(current) > f(z)) {   // can't be >=, o/w the application does not respond
      sigma /= 2;
      current = z - sigma * dz;
      proj_z(current);
  }
  return sigma;
  /////////////////////////////////////////////////////////////////////////////
}
