#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include "RapidTrajectoryGenerator.h"

using namespace RapidQuadrocopterTrajectoryGenerator;

ros::Subscriber future_pva_cmd_sub, uav_odom_sub, goal_sub;
ros::Publisher future_traj_vis_pub, pos_cmd_pub;
ros::Timer cmd_timer;
Eigen::Vector3d pos, vel;
Eigen::Vector3d ref_pos, ref_vel, ref_acc;
Eigen::Vector3d end_pos;
quadrotor_msgs::PositionCommand cmd_msg;

Vec3 pos0, vel0, acc0;
Vec3 posf, velf, accf;
bool have_traj, have_goal;
double last_yaw;

double Tf;
//Define how gravity lies in our coordinate system
Vec3 gravity = Vec3(0, 0, -9.81);//[m/s**2]

Eigen::MatrixXd M(3, 6);
Eigen::MatrixXd T(6, 3);

ros::Time start_time;

void calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

Vec3 EigentoVec3(const Eigen::Vector3d& v)
{
    return Vec3(v(0), v(1), v(2));
}

void setT(double ts, Eigen::MatrixXd& T)
{
    T << pow(ts, 5), 5 * pow(ts, 4), 20 * pow(ts, 3),
         pow(ts, 4), 4 * pow(ts, 3), 12 * pow(ts, 2),
         pow(ts, 3), 3 * pow(ts, 2), 6 * pow(ts, 1),
         pow(ts, 2), 2 * pow(ts, 1),              2,
                 ts,              1,              0,
                  1,              0,              0;
}

void displaySphereList(const std::vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::SPHERE_LIST;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = id;
    future_traj_vis_pub.publish(mk);

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++) 
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        mk.points.push_back(pt);
    }
    future_traj_vis_pub.publish(mk);
    ros::Duration(0.001).sleep();
}

void visualizeTraj()
{
    std::vector<Eigen::Vector3d> traj_pts;
    Eigen::Vector3d pt;

    for (double ts = 0; ts <= Tf; ts += 0.01)
    {
        setT(ts, T);
        pt = M * T.col(0);
        traj_pts.push_back(pt);
    }

    displaySphereList(traj_pts, 0.15, Eigen::Vector4d(0, 0, 0, 1), 10);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::pointMsgToEigen(msg->pose.pose.position, pos);
    tf::vectorMsgToEigen(msg->twist.twist.linear, vel);
}

void wayptCallback(const nav_msgs::Path::ConstPtr& msg)
{
    end_pos << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    auto dp = end_pos - pos;
    last_yaw = atan2(dp(1), dp(0));

    have_goal = true;
}

void futureCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    tf::pointMsgToEigen(msg->position, ref_pos);
    tf::vectorMsgToEigen(msg->velocity, ref_vel);
    tf::vectorMsgToEigen(msg->acceleration, ref_acc);

    posf = EigentoVec3(ref_pos); //position
    velf = EigentoVec3(ref_vel); //velocity
    accf = EigentoVec3(ref_acc); //acceleration

    if(!have_traj)
    {
        start_time = ros::Time::now();

        pos0 = EigentoVec3(pos);
        vel0 = EigentoVec3(vel);
        acc0 = Vec3(0, 0, 0);

        RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(Tf);

        double alpha, beta, gamma;

        for (int dim = 0; dim < 3; dim++)
        {
            alpha = traj.GetAxisParamAlpha(dim);
            beta = traj.GetAxisParamBeta(dim);
            gamma = traj.GetAxisParamGamma(dim);

            M.row(dim) << alpha/120, beta/24, gamma/6, acc0[dim]/2, vel0[dim], pos0[dim];
        }

        have_traj = true;
    }
    else
    {
        ros::Time curr_time = ros::Time::now();
        double curr_t = (curr_time - start_time).toSec();

        if (curr_t > Tf)
        {
            have_traj = false;
            return;
        }

        setT(curr_t, T);
        auto state0 = M * T;

        start_time = ros::Time::now();

        pos0 = EigentoVec3(state0.col(0));
        vel0 = EigentoVec3(state0.col(1));
        acc0 = EigentoVec3(state0.col(2));

        RapidTrajectoryGenerator traj(pos0, vel0, acc0, gravity);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(Tf);

        double alpha, beta, gamma;

        for (int dim = 0; dim < 3; dim++)
        {
            alpha = traj.GetAxisParamAlpha(dim);
            beta = traj.GetAxisParamBeta(dim);
            gamma = traj.GetAxisParamGamma(dim);

            M.row(dim) << alpha/120, beta/24, gamma/6, acc0[dim]/2, vel0[dim], pos0[dim];
        }
    }

    visualizeTraj();
}

void cmdTimerCallback(const ros::TimerEvent& e)
{
    if (!have_traj)
        return;

    ros::Time curr_time = ros::Time::now();
    double curr_t = (curr_time - start_time).toSec();

    if (curr_t <= Tf)
    {
        setT(curr_t, T);
        auto ref_state = M * T;

        cmd_msg.header.stamp = curr_time;
        cmd_msg.header.frame_id = "world";
        cmd_msg.position.x = ref_state(0, 0);
        cmd_msg.position.y = ref_state(1, 0);
        cmd_msg.position.z = ref_state(2, 0);

        cmd_msg.velocity.x = ref_state(0, 1);
        cmd_msg.velocity.y = ref_state(1, 1);
        cmd_msg.velocity.z = ref_state(2, 1);

        cmd_msg.acceleration.x = ref_state(0, 1);
        cmd_msg.acceleration.y = ref_state(1, 1);
        cmd_msg.acceleration.z = ref_state(2, 1);
    }
    else
    {
        setT(Tf, T);
        auto ref_state = M * T;

        cmd_msg.header.stamp = curr_time;
        cmd_msg.header.frame_id = "world";
        cmd_msg.position.x = ref_state(0, 0);
        cmd_msg.position.y = ref_state(1, 0);
        cmd_msg.position.z = ref_state(2, 0);

        cmd_msg.velocity.x = 0;
        cmd_msg.velocity.y = 0;
        cmd_msg.velocity.z = 0;

        cmd_msg.acceleration.x = 0;
        cmd_msg.acceleration.y = 0;
        cmd_msg.acceleration.z = 0;
    }
    Eigen::Vector3d pos_d;
    tf::pointMsgToEigen(cmd_msg.position, pos_d);
    auto dp = end_pos - pos_d;

    // if (dp.norm() < 2)
    // {
    //     tf::pointEigenToMsg(end_pos, cmd_msg.position);
    //     cmd_msg.velocity.x = 0;
    //     cmd_msg.velocity.y = 0;
    //     cmd_msg.velocity.z = 0;

    //     cmd_msg.acceleration.x = 0;
    //     cmd_msg.acceleration.y = 0;
    //     cmd_msg.acceleration.z = 0;   
    // }

    double yaw = atan2(dp(1), dp(0));
    calcNextYaw(last_yaw, yaw);
    last_yaw = yaw;
    cmd_msg.yaw = yaw;

    pos_cmd_pub.publish(cmd_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_gen_node");
    ros::NodeHandle nh("~");

    have_traj = false;
    have_goal = false;

    nh.param("Tf", Tf, 0.2);

    uav_odom_sub = nh.subscribe("/odom", 1, odomCallback);
    future_pva_cmd_sub = nh.subscribe("/future_pva_cmd", 1, futureCmdCallback);
    goal_sub = nh.subscribe("/waypoint_generator/waypoints", 1, wayptCallback);

    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/pva_cmd", 10);
    future_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("/future_traj_vis", 10);

    cmd_timer = nh.createTimer(ros::Duration(0.01), cmdTimerCallback);

    ros::spin();

    return 0;
}