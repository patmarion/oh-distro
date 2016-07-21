#ifndef LCM2ROS_IHMC_CONVERSIONS_HPP_
#define LCM2ROS_IHMC_CONVERSIONS_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>

// TODO: Really really should remove the usage of this function and directly
// use Eigen's own conversions

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw);

#endif /* LCM2ROS_IHMC_CONVERSIONS_HPP_ */