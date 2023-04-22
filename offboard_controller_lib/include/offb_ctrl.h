#pragma once

#include "helper.h"
#include <Eigen/Dense>
#include <vector>

namespace offboard_controller
{
    class OffbCtrl
    {
    private:
        double mass_;
        Eigen::Vector3d gravity_;

        Eigen::Vector3d pos_gain_;
        Eigen::Vector3d vel_gain_;

        std::vector<double> thrust_coeff_, volt_coeff_;
        std::vector<double> thrust_original_; // thrust_original_[0] = norm_thrust_const_
                                              // thrust_original_[1] = norm_thrust_offset_
        double m_a_, m_b_, m_c_;
        double volt_k_, volt_b_;
        double max_acc_;
        double throttle_offset_;
        double throttle_limit_;

    public:
        OffbCtrl(double mass,
                 Eigen::Vector3d pos_gain,
                 Eigen::Vector3d vel_gain,
                 std::vector<double> thrust_coeff,
                 std::vector<double> volt_coeff,
                 double max_acc,
                 double throttle_offset,
                 double throttle_limit,
                 std::vector<double> thrust_original);

        ~OffbCtrl() = default;
        /** @brief parameter setter*/
        // void setMass(double mass); // will not change after set

        void setPosGains(Eigen::Vector3d pos_gain); // may change after set during tuning
        void setVelGains(Eigen::Vector3d vel_gain); // may change after set during tuning

        // void setThrustCoeff(std::vector<double> thrust_coeff); // will not change after set
        // void setVoltCoeff(std::vector<double> volt_coeff); // will not change after set

        void setThrottleOffset(double throttle_offset); // may change after set during tuning

        // void setMaxAcc(double max_acc); // will not change after set

        /** @brief parameter getter*/
        double getMass() { return mass_; };

        Eigen::Vector3d getPosGains() { return pos_gain_; };
        Eigen::Vector3d getVelGains() { return vel_gain_; };

        std::vector<double> getThrustCoeff() { return thrust_coeff_; };
        std::vector<double> getVoltCoeff() { return volt_coeff_; };

        double getThrottleOffset() { return throttle_offset_; };

        double getMaxAcc() { return max_acc_; };

        Eigen::Vector3d calDesiredAcceleration(const Eigen::Vector3d &pos_err, const Eigen::Vector3d &vel_err, const Eigen::Vector3d &acc_cmd);
        Eigen::Vector4d calDesiredAttitude(const Eigen::Vector3d &accel_desired, const double &yaw_ref);
        double calDesiredThrottle(const Eigen::Vector3d &accel_desired, const Eigen::Vector4d &curr_att_quat, const double &battery_volt, const bool voltage_compensation);
    };

} // namespace offboard_controller