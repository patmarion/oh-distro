#include "lcm2ros_ihmc_conversions.hpp"

// TODO: Really really should remove the usage of this function and directly
// use Eigen's own conversions

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw)
{
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  // Post DRC Trails: replace these with Eigen's own conversions
  if (((roll == M_PI) && (pitch == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 1, 0, 0);
  }
  else if (((pitch == M_PI) && (roll == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 0, 1, 0);
  }
  else if (((yaw == M_PI) && (roll == 0)) && (pitch == 0))
  {
    return Eigen::Quaterniond(0, 0, 0, 1);
  }

  double sy = sin(yaw * 0.5);
  double cy = cos(yaw * 0.5);
  double sp = sin(pitch * 0.5);
  double cp = cos(pitch * 0.5);
  double sr = sin(roll * 0.5);
  double cr = cos(roll * 0.5);
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;
  return Eigen::Quaterniond(w, x, y, z);
}