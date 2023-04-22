#include "helper.h"

namespace helper
{
    Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R)
    {
        Eigen::Vector4d quat;
        double tr = R.trace();
        if (tr > 0.0)
        {
            double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
            quat(0) = 0.25 * S;
            quat(1) = (R(2, 1) - R(1, 2)) / S;
            quat(2) = (R(0, 2) - R(2, 0)) / S;
            quat(3) = (R(1, 0) - R(0, 1)) / S;
        }
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
        {
            double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
            quat(0) = (R(2, 1) - R(1, 2)) / S;
            quat(1) = 0.25 * S;
            quat(2) = (R(0, 1) + R(1, 0)) / S;
            quat(3) = (R(0, 2) + R(2, 0)) / S;
        }
        else if (R(1, 1) > R(2, 2))
        {
            double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
            quat(0) = (R(0, 2) - R(2, 0)) / S;
            quat(1) = (R(0, 1) + R(1, 0)) / S;
            quat(2) = 0.25 * S;
            quat(3) = (R(1, 2) + R(2, 1)) / S;
        }
        else
        {
            double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
            quat(0) = (R(1, 0) - R(0, 1)) / S;
            quat(1) = (R(0, 2) + R(2, 0)) / S;
            quat(2) = (R(1, 2) + R(2, 1)) / S;
            quat(3) = 0.25 * S;
        }
        return quat;
    }

    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw)
    {
        //   Eigen::Quaterniond quat;
        Eigen::Vector4d quat;
        Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
        Eigen::Matrix3d rotmat;

        proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

        zb_des = vector_acc / vector_acc.norm();
        yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
        xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

        rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
        //   quat = rotmat;
        quat = rot2Quaternion(rotmat);
        return quat;
    }

    Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q)
    {
        Eigen::Matrix3d rotmat;
        rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
            2 * q(0) * q(2) + 2 * q(1) * q(3),

            2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
            2 * q(2) * q(3) - 2 * q(0) * q(1),

            2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
            q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
        return rotmat;
    }

} // namespace helper