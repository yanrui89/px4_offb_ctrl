#ifndef _PX4_OFFB_CTRL_H_
#define _PX4_OFFB_CTRL_H_

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <px4_offb_ctrl/setInitGlobalPose.h>
#include "offb_ctrl.h"
#include "bspline_utils.hpp"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"


enum UAVState
{
    INIT,
    TAKE_OFF,
    MISSION,
    LAND
};

enum PX4CTRLMode
{
    PVA,
    P,
    V,
    A
};

static const std::string UAVStateStringArr[4] = {"INIT", "TAKE_OFF", "MISSION", "LAND"};

class OffboardNode
{
    ros::NodeHandle _nh;
    ros::Subscriber mav_state_sub;
    ros::Subscriber traj_local_enu_sub;
    ros::Subscriber traj_global_nwu_sub;
    ros::Subscriber uav_local_enu_pose_sub; // Remark: should not use /mavros/local_position/odom for both pose and velocity as the twist in this topic
    ros::Subscriber uav_local_enu_vel_sub;
    ros::Subscriber planner_state_sub;
    ros::Subscriber user_start_sub;
    ros::Publisher att_tgt_pub;
    ros::Publisher pos_tgt_pub;
    ros::Publisher uav_state_pub;
    ros::Publisher uav_global_nwu_pose_pub;
    ros::Publisher uav_global_nwu_odom_pub;
    ros::Publisher _log_path_pub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    ros::ServiceServer reset_init_global_pose_server;

    ros::Subscriber global_nwu_vel_ref_sub;
    ros::Publisher setpoint_vel_pub;
    ros::Publisher setpoint_pos_pub;

    ros::Timer mission_timer;
    ros::Timer tf_listener_timer;
    ros::Timer enu2nwu_tf_timer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    Eigen::Vector3d uav_local_enu_pos;
    Eigen::Vector3d uav_local_enu_vel, uav_local_enu_angvel, uav_global_nwu_vel, uav_global_nwu_angvel;
    Eigen::Affine3d global_to_local_homo, init_local_to_global_homo, init_body_to_local_homo, init_body_to_global_homo;

    mavros_msgs::AttitudeTarget att_target;
    mavros_msgs::PositionTarget pos_sp_target;
    mavros_msgs::State mav_state;

    geometry_msgs::PoseStamped global_nwu_pose_msg;
    nav_msgs::Odometry global_nwu_odom_msg;

    Eigen::Vector3d ref_local_enu_pos, ref_local_enu_vel, ref_local_enu_acc;
    Eigen::Vector3d ref_global_nwu_pos, ref_global_nwu_vel, ref_global_nwu_acc;
    double ref_local_yaw, ref_global_yaw;
    Eigen::Vector4d cmdBodyRate_;

    double mission_timer_interval;
    double takeoff_height;
    bool set_init_local_pose;
    bool reset_init_global_pose;
    bool set_takeoff_ref;
    bool set_offb_success;
    bool start_set_offb_thread;
    bool use_px4_ctrl;
    bool have_pose, have_vel;
    bool use_fastplanner;
    UAVState state_;
    std_msgs::String state_msg;
    int8_t planning_state;
    PX4CTRLMode px4_ctrl_mode;

    std::thread set_offb_thread;

    double battery_volt;
    // Control gains (position, velocity, drag)
    Eigen::Vector3d Kpos_, Kvel_, D_;
    Eigen::Vector3d gravity_;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    std::vector<double> thrust_coeff_, volt_coeff_, thrust_original_;

    bool voltage_compensation_;
    bool using_yawTgt;
    double yaw_ref;
    double max_fb_acc_;
    double attctrl_tau_;
    double norm_thrust_offset_;
    double norm_thrust_const_;
    double mass_; // mass of platform

    double m_a_, m_b_, m_c_, volt_k_, volt_b_;
    double throttle_offset_, throttle_limit_;

    bool user_start;

    Eigen::Vector4d q_des, uav_local_att;

    std::unique_ptr<offboard_controller::OffbCtrl> pos_ctrl;

    /** @brief classes from libbspline packages (functions) that are used in this package **/
    trajectory::bspline_trajectory bsu;
    trajectory::common_trajectory_tool ctt;

    /** @brief Bspline parameters **/
    bool _use_bspline, _bspline_setup, b_spline_completed;
    int _knot_division, _knot_size;
    double _order, _max_velocity;
    double _knot_interval, _duration;
    double _send_command_interval, _send_command_rate;
    double last_yaw;
    Eigen::Quaterniond _takeoff_land_orientation;
    vector<Eigen::Vector3d> wp_pos_vector;
    vector<Eigen::Vector3d> control_points;
    Eigen::Vector3d curr_cp;
    nav_msgs::Path path;


    time_point<std::chrono::system_clock> stime; // start time for bspline server in time_t
    vector<double> timespan;

    /** @brief Initializes the bspline parameters and also sets up the control points **/
    bool initialize_bspline_server(double desired_velocity, Eigen::Vector3d curr_cp, vector<Eigen::Vector3d> wp);

    /** @brief To get the command after the Bspline server has started **/
    bool update_get_command_by_time();

public:
    OffboardNode(ros::NodeHandle& nh);
    ~OffboardNode() = default;

    // ROS callbacks
    void mavStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void uavENUPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uavENUVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void localENUTrajRefCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void globalNWUTrajRefCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void plannerStateCallback(const std_msgs::Int8& msg);
    void missionTimerCallback(const ros::TimerEvent& e);
    void TFListenerTimerCallback(const ros::TimerEvent& e);
    void enu2nwuTimerCallback(const ros::TimerEvent& e);
    bool resetInitGlobalPoseCallback(px4_offb_ctrl::setInitGlobalPose::Request& req,
                                     px4_offb_ctrl::setInitGlobalPose::Response& res);

    void velRefNWUCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void sendCommand();
    void setOffboard();
    void sendPVACtrlCommand();
    void sendPCtrlCommand();
    void sendVCtrlCommand();
    void sendACtrlCommand();
    void sendGeomCtrlCommand();
    void convertGlobal2Local();

    void visualize_log_path();
    geometry_msgs::Point vector_to_point(Eigen::Vector3d v);
    void usrStartCallback(const std_msgs::Bool& msg);

    inline void getStateString(std::string& state)
    {
        state = UAVStateStringArr[(int)state_];
    }

    inline void set_px4_mode(const std::string& mode)
    {
        if (mode == "pva")
            px4_ctrl_mode = PX4CTRLMode::PVA;
        else if (mode == "p")
            px4_ctrl_mode = PX4CTRLMode::P;
        else if (mode == "v")
            px4_ctrl_mode = PX4CTRLMode::V;
        else if (mode == "a")
            px4_ctrl_mode = PX4CTRLMode::A;
        else
            assert(false && "mode is not supported!");
    }


};

#endif