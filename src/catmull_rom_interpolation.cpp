#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  // reference: https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
  Eigen::Vector3d c_t(0, 0, 0);
  int size = keyframes.size();
  if (size == 0) 
      return c_t;

  double local_t = std::fmod(t, keyframes.back().first);    // not sure
  double t_0, t_1, t_2, t_3;
  Eigen::Vector3d m_0, m_2, theta_0, theta_1, theta_2, theta_3;
  int index = 0;
  for (int i = 0; i < size - 1; i++){
      if ((keyframes[i].first <= local_t) && (local_t < keyframes[i + 1].first))
          index = i;
  }
  int temp1 = (index == 0) ? 0 : 1;
  int temp2 = (index == size-2) ? 1 : 0;        // avoid out of range problem 
  t_0 = keyframes[index - temp1].first;
  theta_0 = keyframes[index - temp1].second;

  t_1 = keyframes[index].first;
  theta_1 = keyframes[index].second;

  t_2 = keyframes[index + 1].first;
  theta_2 = keyframes[index + 1].second;

  t_3 = keyframes[index + 2 - temp2].first;
  theta_3 = keyframes[index + 2 - temp2].second;

  m_0 = (theta_2-theta_0) / (t_2-t_0);
  m_2 = (theta_3-theta_1) / (t_3-t_1);  // finite difference 
  double time = (local_t - t_1) / (t_2 - t_1);
  c_t = (2*pow(time,3) - 3*pow(time,2) + 1)*theta_1 + (pow(time,3) - 2*pow(time,2) + time)*m_0 + (-2*pow(time,3) + 3*pow(time,2))*theta_2 + (pow(time,3) - pow(time,2))*m_2;
  return c_t;
  /////////////////////////////////////////////////////////////////////////////
}
