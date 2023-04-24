
#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from px4_offb_ctrl.srv import takeoffExternal, takeoffExternalResponse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import sys
import time

# launch this with
# python send_target.py idx x y z x y z

class TimeHelper:
    """Object containing all time-related functionality.
    This class mainly exists to support both real hardware and (potentially
    faster or slower than realtime) simulation with the same script.
    When running on real hardware, this class uses ROS time functions.
    The simulation equivalent does not depend on ROS.
    Attributes:
        visualizer: No-op object conforming to the Visualizer API used in
            simulation scripts. Maintains the property that scripts should not
            know/care if they are running in simulation or not.
    """
    def __init__(self, node):
        self.node = node
        # self.rosRate = None
        self.rateHz = None
        self.nextTime = None
        # self.visualizer = visNull.VisNull()

    def time(self):
        """Returns the current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        start = self.time()
        end = start + duration
        while self.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        # Note: The following ROS2 construct cannot easily be used, because in ROS2
        #       there is no implicit threading anymore. Thus, the rosRate.sleep() call
        #       is blocking. Instead, we simulate the rate behavior ourselves.
        # if self.rosRate is None or self.rateHz != rateHz:
        #     self.rosRate = self.node.create_rate(rateHz)
        #     self.rateHz = rateHz
        # self.rosRate.sleep()
        if self.nextTime is None or self.rateHz != rateHz:
            self.rateHz = rateHz
            self.nextTime = self.time() + 1.0 / rateHz
        while self.time() < self.nextTime:
            rclpy.spin_once(self.node, timeout_sec=0)
        self.nextTime += 1.0 / rateHz

    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        return not rclpy.ok()

def normalize(v):
  norm = np.linalg.norm(v)
  assert norm > 0
  return v / norm


class Polynomial:
  def __init__(self, p):
    self.p = p

  # evaluate a polynomial using horner's rule
  def eval(self, t):
    assert t >= 0
    x = 0.0
    for i in range(0, len(self.p)):
      x = x * t + self.p[len(self.p) - 1 - i]
    return x

  # compute and return derivative
  def derivative(self):
    return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])


class TrajectoryOutput:
  def __init__(self):
    self.pos = None   # position [m]
    self.vel = None   # velocity [m/s]
    self.acc = None   # acceleration [m/s^2]
    self.omega = None # angular velocity [rad/s]
    self.yaw = None   # yaw angle [rad]


# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial4D:
  def __init__(self, duration, px, py, pz, pyaw):
    self.duration = duration
    self.px = Polynomial(px)
    self.py = Polynomial(py)
    self.pz = Polynomial(pz)
    self.pyaw = Polynomial(pyaw)

  # compute and return derivative
  def derivative(self):
    return Polynomial4D(
      self.duration,
      self.px.derivative().p,
      self.py.derivative().p,
      self.pz.derivative().p,
      self.pyaw.derivative().p)

  def eval(self, t):
    result = TrajectoryOutput()
    # flat variables
    result.pos = np.array([self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
    result.yaw = self.pyaw.eval(t)

    # 1st derivative
    derivative = self.derivative()
    result.vel = np.array([derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
    dyaw = derivative.pyaw.eval(t)

    # 2nd derivative
    derivative2 = derivative.derivative()
    result.acc = np.array([derivative2.px.eval(t), derivative2.py.eval(t), derivative2.pz.eval(t)])

    # 3rd derivative
    derivative3 = derivative2.derivative()
    jerk = np.array([derivative3.px.eval(t), derivative3.py.eval(t), derivative3.pz.eval(t)])

    thrust = result.acc + np.array([0, 0, 9.81]) # add gravity

    z_body = normalize(thrust)
    x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
    y_body = normalize(np.cross(z_body, x_world))
    x_body = np.cross(y_body, z_body)

    jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
    h_w = jerk_orth_zbody / np.linalg.norm(thrust)

    result.omega = np.array([-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
    return result


class Trajectory:
  def __init__(self):
    self.polynomials = None
    self.duration = None

  def n_pieces(self):
    return len(self.polynomials)

  def loadcsv(self, filename):
    data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=range(33))
    self.polynomials = [Polynomial4D(row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]
    self.duration = np.sum(data[:,0])

  def eval(self, t):
    assert t >= 0
    assert t <= self.duration

    current_t = 0.0
    for p in self.polynomials:
      if t <= current_t + p.duration:
        return p.eval(t - current_t)
      current_t = current_t + p.duration


class FlyTrajectory:
    def __init__(self, wp_dir):

        self.cmd_3d = []
        self.traj = Trajectory()
        # self.traj.loadcsv(wp_dir)
        with open(wp_dir,"r") as f:
           self.lines = f.readlines()

        parse_lines = self.parse_txt_file(self.lines)
        self.idx = 0
        self.total_wp = 0

        self.hgt = 1
        self.ate = 30
        self.ready = False
        #rospy.init_node('waypoint_publisher'+drone_id, anonymous=True)
        self.pubOnePt = rospy.Publisher('/trajectory/points', JointTrajectory, latch=True, queue_size=20)
        self.pubTraj = rospy.Publisher('planning/pos_cmd', PositionCommand, latch=True, queue_size=20)
        self.subStart = rospy.Subscriber('/start_publish', Bool, self.callback1 )
        self.subExe = rospy.Subscriber('/start_execute', Bool, self.callback5 )
        self.uav_state = rospy.Subscriber('/mavros/state', State, self.callback5)
        # self.pub_timer = rospy.Timer(rospy.Duration(0.05), self.waypoint_publisher2)
        # self.subPoseWorld = rospy.Subscriber('/drone0/mavros/local_position/pose',self.callback2)
        # self.refGoalWorld = rospy.Subscriber('/drone0/goal',self.callback3)


        #### Service especially for takeoff
        self.takeoffService = rospy.Service('/external_takeoff', takeoffExternal, self.takeoffServiceCallback)

        self.max_vel = 0

        self.start_flight = False

        start_sleep = rospy.Rate(0.5) # 0.5hz

        start_sleep.sleep()

        epoch = rospy.Time.now()
        # print("how aout here")
        # while self.pubOnePt.get_num_connections() < 1:
        #     # Do nothing
        #     print('here?')
        #     if (rospy.Time.now() - epoch).to_sec() > 5.0:
        #         print("No connections in 5s")
        #         return

        # print("or here")

        # drone_name = 'drone'+drone_id
        # jt = JointTrajectory()
        # jt.header.stamp = rospy.Time.now()
        # #drone_name = 'drone'+sys.argv[1]
        # jt.joint_names.append(drone_name)

        # cmd_3d = []
        # cmd_3d.append(0)
        # cmd_3d.append(0)
        # cmd_3d.append(self.hgt)
        # self.waypoint(jt, cmd_3d, (int)(1))

        # self.pubOnePt.publish(jt)
        # print("finished publishing")



        print('completed sending starting point')
        # while self.start_flight == False:
        #     print("still here")
        #     rospy.sleep(1)

        # print("i am here")

        end_sleep = rospy.Rate(1) # 0.5hz

    def takeoffServiceCallback(self, req):
        if req.to_takeoff.data == True:
          print("Received takeoff Service. Proceeding to take off")
          res_msg = Bool()
          res_msg.data = True
          self.ready = True
          return takeoffExternalResponse(res_msg)
        else:
          print("Received takeoff Service. Not taking off")

          res_msg = Bool()
          res_msg.data = False
          return takeoffExternalResponse(res_msg)
        
        
    def parse_txt_file(self, lines):
      total_lines = len(lines)
      pos_list = []
      vel_list = []
      acc_list = []
      for i in range(total_lines):
        curr_line = lines[i]
        curr_line_split_tmp = curr_line.split("\n")
        curr_line_split = curr_line_split_tmp[0].split(" ")
        curr_pos_array = np.zeros(3)
        curr_vel_array = np.zeros(3)
        curr_acc_array =np.zeros(3)
        for j in range(3):
            curr_pos_array[j] = float(curr_line_split[j])
            curr_vel_array[j] = float(curr_line_split[j+3])
            curr_acc_array[j] = float(curr_line_split[j+6])
        
        pos_list.append(curr_pos_array)
        vel_list.append(curr_vel_array)
        acc_list.append(curr_acc_array)

      
      self.pos_array = np.array(pos_list)
      self.vel_array = np.array(vel_list)
      self.acc_array = np.array(acc_list)

      self.total_wp = total_lines
      
      
        
    def callback5(self, data):
        if data.mode == "OFFBOARD" and self.ready == True:
            # time.sleep(2)
            self.waypoint_publisher()
            self.ready = False
        else:
            return


    def waypoint(self,jt, cmd_3d, size):
        
        for i in range(size):
            jtp = JointTrajectoryPoint()
            jtp.positions.append(cmd_3d[i*3+0])
            jtp.positions.append(cmd_3d[i*3+1])
            jtp.positions.append(cmd_3d[i*3+2])
            print(cmd_3d)
            # 7 is trajectory
            jtp.time_from_start = rospy.Duration(7.0)
            jt.points.append(jtp)

        # return jt

    def callback1(self,data):
        self.start_flight = data.data

    # def callback2(self,data):

    def waypoint_publisher(self):
        start_time = rospy.get_time()
        state_msg = PositionCommand()

        for i in range(len(self.lines)):

            t = rospy.get_time() - start_time
            print(t)

            state_msg.header.stamp = rospy.Time.now()
            state_msg.position.x = self.pos_array[i,0]
            state_msg.position.y = self.pos_array[i,1]
            #print(f'The python position for x is {e.pos[0]} and for y is {e.pos[1]}')
            state_msg.position.z = self.pos_array[i,2]
            state_msg.velocity.x = self.vel_array[i,0]
            state_msg.velocity.y = self.vel_array[i,1]
            state_msg.velocity.z = self.vel_array[i,2]
            state_msg.acceleration.x = self.acc_array[i,0]
            state_msg.acceleration.y = self.acc_array[i,1]
            state_msg.acceleration.z = self.acc_array[i,2]
            state_msg.yaw = 0

            print(f'The python position for x is {state_msg.position.x} and for y is {state_msg.position.y} and z is {state_msg.position.z}')


            # print(f'The python absolute velocity is {abs_velocity}')

            # if abs_velocity > self.max_vel:
            #     self.max_vel = abs_velocity

            self.pubTraj.publish(state_msg)
            rospy.sleep(0.05)


def main(wp_dir):
    rospy.init_node('trajectory_generator')
    ft = FlyTrajectory(wp_dir)
    rospy.spin()
    



if __name__ == '__main__':
    waypoint_dir = '/home/yanrui89/storage/catkin_offb/src/px4_offb_ctrl/data/liftoff.txt'
    main(waypoint_dir)