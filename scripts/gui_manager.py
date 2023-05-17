
#!/usr/bin/env python3

import sys
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from nav_msgs.msg import Path
from px4_offb_ctrl.srv import takeoffExternal, takeoffExternalResponse
from mot_msgs.srv import ResetEnvService, ResetEnvServiceRequest
from px4_offb_ctrl.srv import setInitGlobalPose, setInitGlobalPoseRequest
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import sys
import time

# launch this with
# python send_target.py idx x y z x y z

class gui_manager():
    def __init__(self):
        self.takeOffPublisher = rospy.Publisher('/usr_start', Bool, latch=True, queue_size=20)
        self.fastPlannerPublisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True, queue_size=20)
        self.resetEnvClient = rospy.ServiceProxy('/reset_env', ResetEnvService)
        self.resetGlobClient = rospy.ServiceProxy('/reset_init_global_nwu_pose', setInitGlobalPose)

    def takeoff(self):
        print("Sending takeoff")
        takeoffMsg = Bool()
        takeoffMsg.data = True
        self.takeOffPublisher.publish(takeoffMsg)
        time.sleep(2)

    def sendFPWP(self, xyz):
        print("sending fastplanner waypoint")
        xyz_split = xyz.split(" ")
        fpWP = PoseStamped()

        fpWP.header.stamp = rospy.Time.now()
        fpWP.pose.position.x = float(xyz_split[0])
        fpWP.pose.position.y = float(xyz_split[1])
        fpWP.pose.position.z = float(xyz_split[2])

        fpWP.pose.orientation.w = 1.0

        print(fpWP)

        self.fastPlannerPublisher.publish(fpWP)

    def setPlyParam(self):
        print("Setting ply environment")
        rospy.set_param("/global_map_server/map_name", "industrial_mini.ply")

    def resetEnv (self):
        print("resetting environment")
        resetEnvDetails = ResetEnvServiceRequest()
        resetEnvDetails.env_id = 5
        resetEnvDetails.scenario_id = 1
        resetEnvDetails.number_uav = 1

        init_pose = Pose()
        init_pose.position.x = 0.0
        init_pose.position.y = 0.0
        init_pose.position.z = 0.0
        init_pose.orientation.w = 1.0
        

        resetEnvDetails.initial_poses.append(init_pose)

        resetEnvDetails.uav_ids.append(0)

        resetEnvDetails.fish_eye_enabled = True

        print(resetEnvDetails)

        req = self.resetEnvClient(resetEnvDetails)
        
        print(req)


    def resetGlobalPose(self, xyz):
        print("resetting global pose")
        xyz_split = xyz.split(" ")
        resetGlobalPose = setInitGlobalPoseRequest()

        globPose = Pose()

        if len(xyz_split) < 3:
          
          globPose.position.x = 0.0
          globPose.position.y = 0.0
          globPose.position.z = 1.5

        else:
            
          globPose.position.x = float(xyz_split[0])
          globPose.position.y = float(xyz_split[1])
          globPose.position.z = float(xyz_split[2])

        globPose.orientation.w = 1.0

        resetGlobalPose.pose = globPose

        print(globPose)
        req = self.resetGlobClient(resetGlobalPose)


        

        
        
def main():
    rospy.init_node('trajectory_generator')
    gui = gui_manager()
    while not rospy.is_shutdown():
        ans = input("Please choose required task \n 1. Takeoff \n 2. Set fastplanner goal \n 3. Set Ply Environment \n 4. Reset Environment \n 5. Reset global Pose\n")
        if int(ans) == 1:
            gui.takeoff()
        if int(ans) == 2:
            ans1 = input("Please specify x y z coordinate of goal point \n")
            gui.sendFPWP(ans1)
        if int(ans) == 3:
            gui.setPlyParam()
        if int(ans) == 4:
            gui.resetEnv()
        if int(ans) == 5:
            ans2 = input("Specify new global pose to reset to\n")
            gui.resetGlobalPose(ans2)
        
    rospy.spin()
    



if __name__ == '__main__':
    main()