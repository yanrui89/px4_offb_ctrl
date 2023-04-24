#include "px4_offb_ctrl/px4_offb_ctrl.h"

OffboardNode::OffboardNode(ros::NodeHandle& nh) : _nh(nh), tfListener(tfBuffer)
{
    state_ = UAVState::INIT;
    set_init_local_pose = false;
    reset_init_global_pose = false;
    set_takeoff_ref = false;
    set_offb_success = false;
    start_set_offb_thread = false;
    have_pose = false;
    have_vel = false;
    use_fastplanner = false;

    _nh.param<bool>("use_px4_ctrl", use_px4_ctrl, true);
    _nh.param<double>("mission_timer_interval", mission_timer_interval, 0.01);
    _nh.param<double>("takeoff_height", takeoff_height, 2.0);
    _nh.param<bool>("use_fastplanner", use_fastplanner, true);
    set_px4_mode(_nh.param<std::string>("px4_ctrl_mode", "pva"));

    /** @brief B-Spline params*/
    _nh.param<bool>("use_bspline", _use_bspline, true);
    _nh.param<double>("order", _order, 4.0);
    _nh.param<double>("max_velocity", _max_velocity, 2.0);
    _nh.param<int>("knot_division", _knot_division, 3);

    /** @brief External Publisher params*/
    _nh.param<bool>("use_external_pub", _use_external_pub, true);
    _nh.param<bool>("use_external_rel_pub", _use_external_rel_pub, true);

    /** @brief Control gains**/
    _nh.param<double>("gains/p_x", Kpos_x_, 8.0);
    _nh.param<double>("gains/p_y", Kpos_y_, 8.0);
    _nh.param<double>("gains/p_z", Kpos_z_, 10.0);
    _nh.param<double>("gains/v_x", Kvel_x_, 1.5);
    _nh.param<double>("gains/v_y", Kvel_y_, 1.5);
    _nh.param<double>("gains/v_z", Kvel_z_, 3.3);

    _nh.param<bool>("use_yawTarget", using_yawTgt, false);
    _nh.param<double>("max_acc", max_fb_acc_, 9.0);

    _nh.param<double>("attctrl_constant", attctrl_tau_, 0.1);
    _nh.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);
    _nh.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);
    _nh.param<bool>("voltage_compensation", voltage_compensation_, false);

    _nh.param<double>("m_a", m_a_, 202.33);
    _nh.param<double>("m_b", m_b_, 145.56);
    _nh.param<double>("m_c", m_c_, -8.0219);

    _nh.param<double>("volt_k", volt_k_, -0.1088);
    _nh.param<double>("volt_b", volt_b_, 2.1964);

    _nh.param<double>("throttle_offset", throttle_offset_, 0.06);
    _nh.param<double>("throttle_limit", throttle_limit_, 1);

    _nh.param<double>("mass", mass_, 195.5);

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

    thrust_coeff_ = {m_a_, m_b_, m_c_};
    volt_coeff_ = {volt_k_, volt_b_};
    thrust_original_ = {norm_thrust_const_, norm_thrust_offset_};

    gravity_ << 0.0, 0.0, -9.8;

    // B-splines
    _send_command_interval = mission_timer_interval;
    // end B-splines

    // takeoff external
    _takeoff_sent = false;
    user_start = false;

    mav_state_sub = _nh.subscribe("/mavros/state", 1, &OffboardNode::mavStateCallback, this);

    traj_local_enu_sub = _nh.subscribe("/local_enu_traj", 1, &OffboardNode::localENUTrajRefCallback, this);

    traj_global_nwu_sub = _nh.subscribe("/global_nwu_traj", 1, &OffboardNode::globalNWUTrajRefCallback, this);

    uav_local_enu_pose_sub = _nh.subscribe("/mavros/local_position/pose", 1, &OffboardNode::uavENUPoseCallback, this);

    uav_local_enu_vel_sub = _nh.subscribe("/mavros/local_position/velocity_local", 1, &OffboardNode::uavENUVelCallback, this);

    planner_state_sub = _nh.subscribe("/planning/state", 1, &OffboardNode::plannerStateCallback, this);

    user_start_sub = _nh.subscribe("/usr_start", 1, &OffboardNode::usrStartCallback, this);

    store_rel_pose_sub = _nh.subscribe("/usr_store", 1, &OffboardNode::usrStoreRelPoseCallback, this);

    uav_global_nwu_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/global_nwu_pose", 10);

    uav_global_nwu_odom_pub = _nh.advertise<nav_msgs::Odometry>("/global_nwu_odom", 10);

    att_tgt_pub = _nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    pos_tgt_pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    uav_state_pub = _nh.advertise<std_msgs::String>("/uav_state", 10);

    _log_path_pub = _nh.advertise<nav_msgs::Path>("/uav/log_path", 10);

    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    external_takeoff_client = _nh.serviceClient<px4_offb_ctrl::takeoffExternal>("/external_takeoff");

    reset_init_global_pose_server = _nh.advertiseService("/reset_init_global_nwu_pose", &OffboardNode::resetInitGlobalPoseCallback, this);

    mission_timer = _nh.createTimer(ros::Duration(mission_timer_interval), &OffboardNode::missionTimerCallback, this);

    tf_listener_timer = _nh.createTimer(ros::Duration(0.01), &OffboardNode::TFListenerTimerCallback, this);

    enu2nwu_tf_timer = _nh.createTimer(ros::Duration(0.01), &OffboardNode::enu2nwuTimerCallback, this);

    global_nwu_vel_ref_sub = _nh.subscribe("/global_nwu_vel_ref", 1, &OffboardNode::velRefNWUCallback, this);

    setpoint_vel_pub = _nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    setpoint_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    if (!use_px4_ctrl)
    {
        pos_ctrl = std::make_unique<offboard_controller::OffbCtrl>(
            mass_,
            Kpos_,
            Kvel_,
            thrust_coeff_,
            volt_coeff_,
            max_fb_acc_,
            throttle_offset_,
            throttle_limit_,
            thrust_original_);
    }

    init_body_to_local_homo = Eigen::Affine3d::Identity();
    init_body_to_global_homo = Eigen::Affine3d::Identity();
    init_local_to_global_homo = init_body_to_global_homo * init_body_to_local_homo.inverse();
    global_to_local_homo = init_local_to_global_homo.inverse();
}

void OffboardNode::usrStoreRelPoseCallback(const std_msgs::Bool& msg)
{
    user_start = msg.data;
    if (user_start)
    {
        init_rel_local_enu_pos = uav_local_enu_pos;
        ROS_INFO("STORED NEW RELATIVE POSE!!");
    }
    
}


void OffboardNode::usrStartCallback(const std_msgs::Bool& msg)
{
    user_start = msg.data;
}

void OffboardNode::mavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    mav_state = *msg;
}

void OffboardNode::plannerStateCallback(const std_msgs::Int8& msg)
{
    planning_state = msg.data;
}

void OffboardNode::uavENUPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!have_pose)
        have_pose = true;

    tf::pointMsgToEigen(msg->pose.position, uav_local_enu_pos);
    uav_local_att << msg->pose.orientation.w,
                     msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "local_enu";
    transformStamped.child_frame_id = "body";
    transformStamped.transform.translation.x = uav_local_enu_pos.x();
    transformStamped.transform.translation.y = uav_local_enu_pos.y();
    transformStamped.transform.translation.z = uav_local_enu_pos.z();
    transformStamped.transform.rotation.x = uav_local_att(1);
    transformStamped.transform.rotation.y = uav_local_att(2);
    transformStamped.transform.rotation.z = uav_local_att(3);
    transformStamped.transform.rotation.w = uav_local_att(0);

    br.sendTransform(transformStamped);   
}

void OffboardNode::convertGlobal2Local()
{
    tf2::Quaternion traj_nwu_q;
    traj_nwu_q.setRPY(0, 0, ref_global_yaw);
    Eigen::Affine3d traj_nwu_homo = Eigen::Affine3d::Identity();

    traj_nwu_homo.translation() = ref_global_nwu_pos;
    traj_nwu_homo.linear() = Eigen::Quaterniond(traj_nwu_q.getW(),
                                                traj_nwu_q.getX(),
                                                traj_nwu_q.getY(),
                                                traj_nwu_q.getZ()).toRotationMatrix();
    
    Eigen::Affine3d traj_enu_homo = global_to_local_homo * traj_nwu_homo;
    ref_local_enu_pos = traj_enu_homo.translation();
    Eigen::Quaterniond traj_enu_q(traj_enu_homo.linear());
    tf2::Quaternion traj_enu_tf2_q(traj_enu_q.x(),
                                   traj_enu_q.y(),
                                   traj_enu_q.z(),
                                   traj_enu_q.w());
    tf2::Matrix3x3 traj_enu_tf2_rot(traj_enu_tf2_q);
    double r, p, y;
    traj_enu_tf2_rot.getRPY(r, p, y);
    ref_local_yaw = y;

    ref_local_enu_vel = global_to_local_homo.linear() * ref_global_nwu_vel;
    ref_local_enu_acc = global_to_local_homo.linear() * ref_global_nwu_acc;
}

void OffboardNode::uavENUVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if (!have_vel)
        have_vel = true;

    tf::vectorMsgToEigen(msg->twist.linear, uav_local_enu_vel);
    tf::vectorMsgToEigen(msg->twist.angular, uav_local_enu_angvel);
}

void OffboardNode::velRefNWUCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if (reset_init_global_pose)
        reset_init_global_pose = false;

    tf::vectorMsgToEigen(msg->twist.linear, ref_global_nwu_vel);
    convertGlobal2Local();
}

void OffboardNode::globalNWUTrajRefCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if (reset_init_global_pose)
    {
        if (use_fastplanner)
        {
            if (planning_state == 0 || planning_state == 1)
                return;
        }

        reset_init_global_pose = false;
    }

    tf::pointMsgToEigen(msg->position, ref_global_nwu_pos);
    tf::vectorMsgToEigen(msg->velocity, ref_global_nwu_vel);
    tf::vectorMsgToEigen(msg->acceleration, ref_global_nwu_acc);
    ref_global_yaw = msg->yaw;
    convertGlobal2Local();
}

void OffboardNode::localENUTrajRefCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if (_use_external_rel_pub)
    {

        std::cout << " I am inside here\n";
        tf2::Quaternion traj_enu_tf2_q(uav_local_att(1),
                                uav_local_att(2),
                                uav_local_att(3),
                                uav_local_att(0));
        tf2::Matrix3x3 traj_enu_tf2_rot(traj_enu_tf2_q);
        double r, p, y;
        traj_enu_tf2_rot.getRPY(r, p, y);
        ref_local_yaw = y;
        tf::pointMsgToEigen(msg->position, ref_local_enu_pos_tmp);
        for (int i = 0; i < 3; i++)
        {
            ref_local_enu_pos[i] = ref_local_enu_pos_tmp[i] + init_rel_local_enu_pos[i];
        }
        tf::vectorMsgToEigen(msg->velocity, ref_local_enu_vel);
        tf::vectorMsgToEigen(msg->acceleration, ref_local_enu_acc);
        
    }

    else
    {
        tf::pointMsgToEigen(msg->position, ref_local_enu_pos);
        tf::vectorMsgToEigen(msg->velocity, ref_local_enu_vel);
        tf::vectorMsgToEigen(msg->acceleration, ref_local_enu_acc);
        ref_local_yaw = msg->yaw;
    }

}

bool OffboardNode::resetInitGlobalPoseCallback(px4_offb_ctrl::setInitGlobalPose::Request& req,
                                               px4_offb_ctrl::setInitGlobalPose::Response& res)
{
    init_body_to_global_homo.translation() = Eigen::Vector3d(req.pose.position.x,
                                                             req.pose.position.y,
                                                             req.pose.position.z);

    init_body_to_global_homo.linear() = Eigen::Quaterniond(req.pose.orientation.w,
                                                           req.pose.orientation.x,
                                                           req.pose.orientation.y,
                                                           req.pose.orientation.z).toRotationMatrix();
    
    init_body_to_local_homo.translation() = uav_local_enu_pos;

    init_body_to_local_homo.linear() = Eigen::Quaterniond(uav_local_att(0),
                                                          uav_local_att(1),
                                                          uav_local_att(2),
                                                          uav_local_att(3)).toRotationMatrix();

    init_local_to_global_homo = init_body_to_global_homo * init_body_to_local_homo.inverse();
    global_to_local_homo = init_local_to_global_homo.inverse();

    reset_init_global_pose = true;

    std_msgs::Bool res_msg;
    res_msg.data = true;
    res.success = res_msg;

    return true;
}

void OffboardNode::TFListenerTimerCallback(const ros::TimerEvent& e)
{
    geometry_msgs::TransformStamped body2global_transformStamped;
    try
    {
        body2global_transformStamped = tfBuffer.lookupTransform("global_nwu", "body", ros::Time(0));
    }
    catch(const tf2::TransformException& e)
    {
        ROS_WARN("%s", e.what());
        ros::Duration(1.0).sleep();
        return;
    }

    global_nwu_pose_msg.header.stamp = ros::Time::now();
    global_nwu_odom_msg.header.frame_id = "global_nwu";
    global_nwu_pose_msg.pose.position.x = body2global_transformStamped.transform.translation.x;
    global_nwu_pose_msg.pose.position.y = body2global_transformStamped.transform.translation.y;
    global_nwu_pose_msg.pose.position.z = body2global_transformStamped.transform.translation.z;
    global_nwu_pose_msg.pose.orientation.w = body2global_transformStamped.transform.rotation.w;
    global_nwu_pose_msg.pose.orientation.x = body2global_transformStamped.transform.rotation.x;
    global_nwu_pose_msg.pose.orientation.y = body2global_transformStamped.transform.rotation.y;
    global_nwu_pose_msg.pose.orientation.z = body2global_transformStamped.transform.rotation.z;
    uav_global_nwu_pose_pub.publish(global_nwu_pose_msg);

    global_nwu_odom_msg.header.stamp = ros::Time::now();
    global_nwu_odom_msg.header.frame_id = "global_nwu";
    global_nwu_odom_msg.child_frame_id = "body";
    global_nwu_odom_msg.pose.pose.position.x = body2global_transformStamped.transform.translation.x;
    global_nwu_odom_msg.pose.pose.position.y = body2global_transformStamped.transform.translation.y;
    global_nwu_odom_msg.pose.pose.position.z = body2global_transformStamped.transform.translation.z;
    global_nwu_odom_msg.pose.pose.orientation.w = body2global_transformStamped.transform.rotation.w;
    global_nwu_odom_msg.pose.pose.orientation.x = body2global_transformStamped.transform.rotation.x;
    global_nwu_odom_msg.pose.pose.orientation.y = body2global_transformStamped.transform.rotation.y;
    global_nwu_odom_msg.pose.pose.orientation.z = body2global_transformStamped.transform.rotation.z;

    uav_global_nwu_vel = init_local_to_global_homo.linear() * uav_local_enu_vel;
    global_nwu_odom_msg.twist.twist.linear.x = uav_global_nwu_vel.x();
    global_nwu_odom_msg.twist.twist.linear.y = uav_global_nwu_vel.y();
    global_nwu_odom_msg.twist.twist.linear.z = uav_global_nwu_vel.z();

    uav_global_nwu_angvel = init_local_to_global_homo.linear() * uav_local_enu_angvel;
    global_nwu_odom_msg.twist.twist.angular.x = uav_global_nwu_angvel.x();
    global_nwu_odom_msg.twist.twist.angular.y = uav_global_nwu_angvel.y();
    global_nwu_odom_msg.twist.twist.angular.z = uav_global_nwu_angvel.z();

    uav_global_nwu_odom_pub.publish(global_nwu_odom_msg);
}

void OffboardNode::enu2nwuTimerCallback(const ros::TimerEvent& e)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "global_nwu";
    transformStamped.child_frame_id = "local_enu";
    transformStamped.transform.translation.x = init_local_to_global_homo.translation().x();
    transformStamped.transform.translation.y = init_local_to_global_homo.translation().y();
    transformStamped.transform.translation.z = init_local_to_global_homo.translation().z();
    Eigen::Quaterniond enu2nwu_rot(init_local_to_global_homo.linear());
    transformStamped.transform.rotation.x = enu2nwu_rot.x();
    transformStamped.transform.rotation.y = enu2nwu_rot.y();
    transformStamped.transform.rotation.z = enu2nwu_rot.z();
    transformStamped.transform.rotation.w = enu2nwu_rot.w();

    br.sendTransform(transformStamped);
}

void OffboardNode::missionTimerCallback(const ros::TimerEvent& e)
{
    getStateString(state_msg.data);
    uav_state_pub.publish(state_msg);

    switch (state_)
    {
        case UAVState::INIT:
        {


            if (!have_pose || !have_vel)
                return;

            if (!set_init_local_pose)
            {
                ref_local_enu_pos << uav_local_enu_pos.x(), 
                                     uav_local_enu_pos.y(),
                                     uav_local_enu_pos.z();

                ref_local_enu_vel.setZero();
                ref_local_enu_acc.setZero();

                tf2::Quaternion q(
                    uav_local_att(1),
                    uav_local_att(2),
                    uav_local_att(3),
                    uav_local_att(0)
                );

                tf2::Matrix3x3 m(q);
                double r, p, y;
                m.getRPY(r, p, y);
                ref_local_yaw = y;

                set_init_local_pose = true;
            }

            if (!user_start){
                return;
            }           

            if (!start_set_offb_thread && !set_offb_success)
            {
                ROS_INFO("Start set offboard thread!");
                set_offb_thread = std::thread(&OffboardNode::setOffboard, this);
                start_set_offb_thread = true;
            }


            if (set_offb_success)
            {
                if (_use_bspline)
                {
                    _bspline_setup = false;
                    while (!_bspline_setup){
                        wp_pos_vector.clear();
                        curr_cp = Eigen::Vector3d(ref_local_enu_pos.x(),
						                            ref_local_enu_pos.y(), ref_local_enu_pos.z());
                        ref_local_enu_pos.z() += takeoff_height;
                        wp_pos_vector.push_back(Eigen::Vector3d(
                                                            ref_local_enu_pos.x(),
                                                            ref_local_enu_pos.y(), 
                                                            ref_local_enu_pos.z()));
                        _bspline_setup = initialize_bspline_server(0.3, curr_cp, wp_pos_vector);
                        wp_pos_vector.clear();

                        tf2::Quaternion traj_enu_tf2_q(uav_local_att(1),
                                                        uav_local_att(2),
                                                        uav_local_att(3),
                                                        uav_local_att(0));
                        tf2::Matrix3x3 traj_enu_tf2_rot(traj_enu_tf2_q);
                        double r, p, y;
                        traj_enu_tf2_rot.getRPY(r, p, y);
                        last_yaw = y;

                    }
                    printf("[%soffboardnode.cpp] %skTakeoff/kLand %sFinished setting up bspline!%s \n", 
					        KGRN, KNRM, KBLU, KNRM); 
                }
                else if (_use_external_pub)
                {
                    ref_local_enu_pos.z() = uav_local_enu_pos.z();
                }
                else
                {
                    ref_local_enu_pos.z() += takeoff_height;
                }
                stime = std::chrono::system_clock::now();

                state_ = UAVState::TAKE_OFF;
                ROS_INFO("The vehicle will take off!");             
            }
            break;
        }
        case UAVState::TAKE_OFF:
        {
            if (!mav_state.armed)
            {
                ROS_ERROR("The Vehicle is not armed, please arm the vehicle!");
            }

            sendCommand();
            if (_use_bspline){
                if (b_spline_completed){
                    ROS_INFO("Takeoff completed!");
                    state_ = UAVState::MISSION;
                }
            }
            else if (_use_external_pub)
            {
                if (!_takeoff_sent)
                {
                    std_msgs::Bool tmp_takeoff_param;
                    std_msgs::Bool _takeoff_sent_tmp;
                    tmp_takeoff_param.data = true;
                    takeoff_srv.request.to_takeoff = tmp_takeoff_param;
                    if (external_takeoff_client.call(takeoff_srv))
                    {
                        ROS_INFO("External take off service sent!");
                        _takeoff_sent = takeoff_srv.response.success.data;
                    }
                    else
                    {
                        ROS_ERROR("Failed to call takeoff service. Trying again");
                    }

                    return;

                }
                if (std::abs(uav_local_enu_pos.z() - takeoff_height) < 0.05)
                {
                    ROS_INFO("Takeoff completed!");
                    state_ = UAVState::MISSION;
                }
            }
            else{
                if (std::abs(uav_local_enu_pos.z() - ref_local_enu_pos.z()) < 0.05)
                {
                    ROS_INFO("Takeoff completed!");
                    state_ = UAVState::MISSION;
                }

            }

            break;
        }
        case UAVState::MISSION:
        {
            if (reset_init_global_pose)
            {
                ref_global_nwu_pos = init_body_to_global_homo.translation();
                ref_global_nwu_vel.setZero();
                ref_global_nwu_acc.setZero();
                Eigen::Quaterniond init_global_nwu_att(init_body_to_global_homo.linear());
                tf2::Quaternion init_tf2_nwu_att(init_global_nwu_att.x(), 
                                                init_global_nwu_att.y(), 
                                                init_global_nwu_att.z(), 
                                                init_global_nwu_att.w());
                tf2::Matrix3x3 init_tf2_nwu_rot(init_tf2_nwu_att);
                double r, p, y;
                init_tf2_nwu_rot.getRPY(r, p, y);
                ref_global_yaw = y;
                convertGlobal2Local();
            }

            sendCommand();
        }
        default:
            break;
    }
}

void OffboardNode::setOffboard()
{
    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !mav_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        sendCommand();
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (!set_offb_success)
    {
        if (mav_state.mode != "OFFBOARD" && 
        ros::Time::now() - last_request > ros::Duration(1.0))
        {
            ROS_INFO("Try set offboard!");
            if (set_mode_client.call(offb_set_mode) && 
            offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard mode enabled!");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!mav_state.armed && 
            ros::Time::now() - last_request > ros::Duration(1.0))
            {
                ROS_INFO("Try arming!");
                if (arming_client.call(arm_cmd) && 
                arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed!");
                }
                last_request = ros::Time::now();
            }
        }
        sendCommand();
        set_offb_success = (mav_state.mode == "OFFBOARD" && mav_state.armed);
        ros::spinOnce();
        rate.sleep();
    }
}

void OffboardNode::sendPVACtrlCommand()
{
    // use PX4 by setting ref p, v, a, yaw
    pos_sp_target.header.stamp = ros::Time::now();
    pos_sp_target.header.frame_id = "local_enu";
    pos_sp_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // TODO: check whether this is necessary
    tf::pointEigenToMsg(ref_local_enu_pos, pos_sp_target.position);
    tf::vectorEigenToMsg(ref_local_enu_vel, pos_sp_target.velocity);
    tf::vectorEigenToMsg(ref_local_enu_acc, pos_sp_target.acceleration_or_force);
    pos_sp_target.yaw = ref_local_yaw;

    // http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html
    pos_sp_target.type_mask = 2048; // use p,v,a and ignore yaw_rate
    pos_tgt_pub.publish(pos_sp_target);
}

void OffboardNode::sendPCtrlCommand()
{
    geometry_msgs::PoseStamped pos_cmd;
    pos_cmd.header.stamp = ros::Time::now();
    pos_cmd.header.frame_id = "local_enu";
    tf::pointEigenToMsg(ref_local_enu_pos, pos_cmd.pose.position);

    setpoint_pos_pub.publish(pos_cmd);
}

void OffboardNode::sendVCtrlCommand()
{
    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.header.frame_id = "local_enu";
    tf::vectorEigenToMsg(ref_local_enu_vel, vel_cmd.twist.linear);

    setpoint_vel_pub.publish(vel_cmd);
}

void OffboardNode::sendGeomCtrlCommand()
{
    auto pos_error = uav_local_enu_pos - ref_local_enu_pos;
    auto vel_error = uav_local_enu_vel - ref_local_enu_vel;

    auto a_des = pos_ctrl->calDesiredAcceleration(pos_error, vel_error, ref_local_enu_acc);
    q_des = pos_ctrl->calDesiredAttitude(a_des, ref_local_yaw); // call acc2quaternion under the hood
    cmdBodyRate_(3) = pos_ctrl->calDesiredThrottle(a_des, uav_local_att, battery_volt, voltage_compensation_);

    att_target.header.stamp = ros::Time::now();
    att_target.header.frame_id = "local_enu";
    att_target.body_rate.x = cmdBodyRate_(0);
    att_target.body_rate.y = cmdBodyRate_(1);
    att_target.body_rate.z = cmdBodyRate_(2);
    // http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html
    att_target.type_mask = 7; // Ignore orientation messages (128); Ignore body rate messages (7)
    att_target.orientation.w = q_des(0);
    att_target.orientation.x = q_des(1);
    att_target.orientation.y = q_des(2);
    att_target.orientation.z = q_des(3);
    att_target.thrust = cmdBodyRate_(3);

    att_tgt_pub.publish(att_target);
}

void OffboardNode::sendCommand()
{

    if (state_ == UAVState::TAKE_OFF)
    {
        
        if (_use_bspline)
        {
            if (update_get_command_by_time()){
                sendPVACtrlCommand();
                return;
            }
        }
        else
        {
            sendPVACtrlCommand();
            // sendGeomCtrlCommand();
            return;
        }

        
    }

    if (use_px4_ctrl)
    {
        
        switch (px4_ctrl_mode)
        {           
            case PX4CTRLMode::PVA:
            {
                sendPVACtrlCommand();
                break;
            }
            case PX4CTRLMode::P:
            {
                sendPCtrlCommand();
                break;
            }
            case PX4CTRLMode::V:
            {
                sendVCtrlCommand();
                break;
            }
            default:
                break;
        }
    }
    else
    {
        sendGeomCtrlCommand();
    }
}

bool OffboardNode::initialize_bspline_server(double desired_velocity, Eigen::Vector3d curr_cp, vector<Eigen::Vector3d> wp_pos_vector)
{
	b_spline_completed = false;
    path.poses.clear();
	double total_distance = 0.0;
	for (int i = 0; i < wp_pos_vector.size(); i++)
		total_distance += wp_pos_vector[i].norm();
	
	double est_duration = total_distance / desired_velocity;

	// Re-adjust duration so that our knots and divisions are matching
	double corrected_duration_secs = bsu.get_corrected_duration(
		_send_command_interval, est_duration);

	_knot_size = bsu.get_knots_size(
            _send_command_interval, corrected_duration_secs, _knot_division);

	_knot_interval = corrected_duration_secs / _knot_size;

	control_points.clear();
	control_points = ctt.uniform_distribution_of_cp(
        curr_cp, wp_pos_vector, 
		_max_velocity, _knot_interval);
	
	// clamp start and also the end to stop uav at the beginning and the end
	for (int i = 0; i < _order; i++)
	{
		control_points.insert(control_points.begin(), 1, control_points[0]);
		control_points.push_back(control_points[control_points.size()-1]);
	}

	_duration = ((double)control_points.size() + (double)_order) * _knot_interval;

    std::cout << " CHECK OUT THE DURATIONNNNNNNNNNNNNNNN\n" << _duration;
    timespan.clear();
    timespan.push_back(0.0);
    timespan.push_back(_duration);

    return true;
}


bool OffboardNode::update_get_command_by_time()
{
	// Bspline is updated before we reach here;
	// bs_control_points = cp;

	time_point<std::chrono::system_clock> now_time = 
		system_clock::now();

	double rel_now_time = duration<double>(now_time - stime).count();
	if ((timespan[1] - rel_now_time) < 0)
	{
		std::cout << "[Offboardnode.cpp] rel_now_time is outside of timespan[1]" << KNRM << std::endl;
        b_spline_completed = true;
        visualize_log_path();
		return false;
	}

	trajectory::bspline_trajectory::bs_pva_state_3d pva3;
	pva3 = bsu.get_single_bspline_3d(
		_order, timespan, control_points, rel_now_time);

    
    ref_local_enu_pos = pva3.pos[0];
	// cmd_nwu.pos = pva3.pos[0];
	// cmd_nwu.t = rel_now_time;
	
	// if (!pva3.vel.empty())
    ref_local_enu_vel = pva3.vel[0];
	double _norm = sqrt(pow(ref_local_enu_vel.x(),2) + pow(ref_local_enu_vel.y(),2));
	double _norm_x = ref_local_enu_vel.x() / _norm;
	double _norm_y = ref_local_enu_vel.y() / _norm;

	if (!pva3.acc.empty())
		ref_local_enu_acc = pva3.acc[0];
	
	// If velocity is too low, then any noise will cause the yaw to fluctuate
	// Restrict the yaw if velocity is too low
	if (ref_local_enu_vel.norm() >= 0.15)
		last_yaw = atan2(_norm_y,_norm_x);

	ref_local_yaw = last_yaw;

	return true;
}


void OffboardNode::visualize_log_path()
{
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "global_nwu";

	for (int i = 0; i < control_points.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "global_nwu";
		pose.pose.position = vector_to_point(control_points[i]);
		path.poses.push_back(pose);
	}

	_log_path_pub.publish(path);

}

/** @brief vector_to_point Eigen to Ros **/
geometry_msgs::Point OffboardNode::vector_to_point(Eigen::Vector3d v)
{
    geometry_msgs::Point tmp;
    tmp.x = v.x(); 
    tmp.y = v.y(); 
    tmp.z = v.z();

    return tmp;
}
