#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  double h = 1.0e-7;
  Eigen::VectorXd tips = transformed_tips(skeleton, b);
  Eigen::VectorXd new_tips;
  for (int i = 0; i < skeleton.size(); i++) {
      for (int j = 0; j < 3; j++) {
          Skeleton copy = skeleton;   // avoid changing the input skeleton
          copy[i].xzx[j] += h;
          new_tips = transformed_tips(copy, b);
          for (int k = 0; k < J.rows(); k++) 
              J(k, 3 * i +j) = (new_tips - tips)[k] / h;
      }
  }
  /////////////////////////////////////////////////////////////////////////////
}
