#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include "Skeleton.h"
#include "Bone.h"
#include <functional> // std::function

// recursive helper function 
Eigen::Affine3d helper(Bone current, const Skeleton & skeleton){
  if(current.parent_index == -1)
      return Eigen::Affine3d::Identity();

  Eigen::Affine3d Tp = helper(skeleton[current.parent_index], skeleton);
  Eigen::Affine3d deltaTi = euler_angles_to_transform(current.xzx);
  return Tp * current.rest_T * deltaTi * current.rest_T.inverse();

}



void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  for(int i = 0; i < skeleton.size(); i++)
    T[i] = helper(skeleton[i], skeleton);
  /////////////////////////////////////////////////////////////////////////////
}
