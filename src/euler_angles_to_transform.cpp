#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double pi = 3.14159;
  double x1 = xzx.x() * pi/180;
  double z = xzx.y() * pi/180;
  double x2 = xzx.z() * pi/180;

  Eigen::Affine3d final, twist_x2, twist_z, twist_x1;
  twist_x1.matrix() << 
    1,0,0,0,
    0,cos(x1),-sin(x1),0,
    0,sin(x1),cos(x1),0,
    0,0,0,1;

  twist_z.matrix() << 
    cos(z),-sin(z),0,0,
    sin(z),cos(z),0,0,
    0,0,1,0,
    0,0,0,1;

  twist_x2.matrix() << 
    1,0,0,0,
    0,cos(x2),-sin(x2),0,
    0,sin(x2),cos(x2),0,
    0,0,0,1;

  final = twist_x2*twist_z*twist_x1;
  return final;
  /////////////////////////////////////////////////////////////////////////////
}
