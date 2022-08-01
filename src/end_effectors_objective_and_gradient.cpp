#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tips = transformed_tips(copy, b);
    return (tips - xb0).squaredNorm();  // check 
  };

  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd tips = transformed_tips(copy, b);
    Eigen::MatrixXd jacobian;
    kinematics_jacobian(copy, b, jacobian);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(A.size());
    for(int i = 0; i < b.size(); i++){
      for(int j = 0; j < 3; j++)
        gradient += 2 * (tips[3*i + j]-xb0[3*i + j]) * jacobian.row(3 * i + j).transpose();
    }
    return gradient;
  };
  
  proj_z = [&](Eigen::VectorXd & A)
  {
    assert(skeleton.size()*3 == A.size());
    for(int i = 0; i < skeleton.size(); i++){
      for(int j = 0; j < 3; j++)
        A[3*i + j] = std::max(skeleton[i].xzx_min[j], std::min(skeleton[i].xzx_max[j], A[3*i + j]));
    }
  };
  /////////////////////////////////////////////////////////////////////////////
}
