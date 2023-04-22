#include "offb_ctrl.h"
namespace offboard_controller
{
    OffbCtrl::OffbCtrl(double mass,
                       Eigen::Vector3d pos_gain,
                       Eigen::Vector3d vel_gain,
                       std::vector<double> thrust_coeff,
                       std::vector<double> volt_coeff,
                       double max_acc,
                       double throttle_offset,
                       double throttle_limit,
                       std::vector<double> thrust_original) :

                                                              mass_(mass),
                                                              pos_gain_(pos_gain),
                                                              vel_gain_(vel_gain),
                                                              thrust_coeff_(thrust_coeff),
                                                              volt_coeff_(volt_coeff),
                                                              max_acc_(max_acc),
                                                              throttle_offset_(throttle_offset),
                                                              throttle_limit_(throttle_limit),
                                                              thrust_original_(thrust_original)
    {
        gravity_ = Eigen::Vector3d{0.0, 0.0, -9.81};
        m_a_ = thrust_coeff[0];
        m_b_ = thrust_coeff[1];
        m_c_ = thrust_coeff[2];
        volt_k_ = volt_coeff[0];
        volt_b_ = volt_coeff[1];
    }

    void OffbCtrl::setPosGains(Eigen::Vector3d pos_gain)
    {
        this->pos_gain_ = pos_gain;
    } // may change after set during tuning

    void OffbCtrl::setVelGains(Eigen::Vector3d vel_gain)
    {
        this->vel_gain_ = vel_gain;
    } // may change after set during tuning

    void OffbCtrl::setThrottleOffset(double throttle_offset)
    {
        this->throttle_offset_ = throttle_offset;
    } // may change after set during tuning

    Eigen::Vector3d OffbCtrl::calDesiredAcceleration(const Eigen::Vector3d &pos_err, const Eigen::Vector3d &vel_err, const Eigen::Vector3d &acc_cmd)
    {
        Eigen::Vector3d a_fb = pos_gain_.asDiagonal() * pos_err + vel_gain_.asDiagonal() * vel_err;

        if (a_fb.norm() > max_acc_)
        {
            a_fb = (max_acc_ / a_fb.norm()) * a_fb;
        }

        Eigen::Vector3d acc_desired = a_fb + acc_cmd - gravity_;

        return acc_desired;
    }

    Eigen::Vector4d OffbCtrl::calDesiredAttitude(const Eigen::Vector3d &accel_desired, const double &yaw_ref)
    {
        return helper::acc2quaternion(accel_desired, yaw_ref);
    }

    double OffbCtrl::calDesiredThrottle(const Eigen::Vector3d &accel_desired, const Eigen::Vector4d &curr_att_quat, const double &battery_volt, const bool voltage_compensation)
    {
        Eigen::Matrix3d rotmat = helper::quat2RotMatrix(curr_att_quat);
        const Eigen::Vector3d zb = rotmat.col(2);
        double acc_body = accel_desired.dot(zb);
        double throttle = thrust_original_[0] * acc_body + thrust_original_[1];

        if (voltage_compensation)
        {
            double force_gram_per_motor = acc_body * mass_ / 9.81 / 4; // convert to force in grams per motor
            double cmd_theoretical = (-m_b_ + sqrt(pow(m_b_, 2) - 4 * m_a_ * (m_c_ - force_gram_per_motor))) / (2 * m_a_);
            double ratio = battery_volt * volt_k_ + volt_b_;
            throttle = (cmd_theoretical - throttle_offset_) * ratio;
        }

        return std::max(0.0, std::min(throttle_limit_, throttle));  // Limit throttle
    }

} // namespace offboard_controller
