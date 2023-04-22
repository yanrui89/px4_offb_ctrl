#pragma once
#include <Eigen/Dense>

namespace helper
{

  Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);

  Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

  Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);

} // namespace helper
