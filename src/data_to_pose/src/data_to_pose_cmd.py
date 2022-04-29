#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from time import time
from geometry_msgs.msg import Vector3, Quaternion, Pose
from tf import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
from rospy import Time
from data_to_pose.msg import ME439WaypointXYZ
from std_msgs.msg import Bool




class data2Pose:
    def __init__(self):
        self.pose_msg = ME439WaypointXYZ()
        self.path_complete = Bool()

        self.translation = (0.0, 0.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.rate = rospy.Rate(500)

        self.pos = [0,0,0]

        self.dt = 0.01

        self.angVelx = 0
        self.angVely = 0
        self.angVelz = 0

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q0 = 1

        self.pose_msg.xyz[0] =0.2015
        self.pose_msg.xyz[1] =0.0
        self.pose_msg.xyz[2] = 0.2057

    def orientationCallback(self, msg):
        # "Store" message received.
        self.q1 = msg.x
        self.q2 = msg.y
        self.q3 = msg.z
        self.q0 = msg.w
        # Compute stuff.
        self.compute_stuff()

    def velocityCallback(self, msg):
        # "Store" the message received.
        self.angVelx = msg.x
        self.angVely = msg.y
        self.angVelz = msg.z

        # Compute stuff.
        self.compute_stuff()
    
    def joyCallback(self, msg):
        # "Store" the message received.
        joyX = msg.data[0]
        joyY = -msg.data[1]
        joyZ = msg.data[2]
        self.joy = [joyX,joyY,joyZ]
        vel = [0,0,0]
        for i in range(2):
            if abs(self.joy[i]) < 15:
                self.joy[i] = 0
            else:
                self.joy[i] = self.joy[i]*0.00005
            
        self.joy[2] =self.joy[2]*0.005
        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):

        #rotation = (-self.q1, -self.q2, -self.q3, self.q0)
        rotation = (self.q1, self.q2, self.q3, self.q0)
        r = R.from_quat(np.array([self.q1, self.q2, self.q3, self.q0]))
        r_pos = r.apply(np.array(self.joy))
        
        self.pose_msg.xyz[0] +=r_pos[0]*self.dt
        self.pose_msg.xyz[1] +=r_pos[1]*self.dt
        self.pose_msg.xyz[2] +=r_pos[2]*self.dt
        
        for i in range(2):
            if self.pose_msg.xyz[i]>=.25:
                self.pose_msg.xyz[i] =.25
            elif self.pose_msg.xyz[i]<=-.25:
                self.pose_msg.xyz[i] =-.25

        if self.pose_msg.xyz[2]>=.25:
            self.pose_msg.xyz[2] =.25
        elif self.pose_msg.xyz[2]<=0:
            self.pose_msg.xyz[2] =0


        # self.pose_msg.orientation.x = 0.
        # self.pose_msg.orientation.y = 0.
        # self.pose_msg.orientation.z = 0.
        # self.pose_msg.orientation.w = 1.

        pose_pub.publish(self.pose_msg)
        self.path_complete.data = False
        path_pub.publish(self.path_complete.data)
        



if __name__ == '__main__':
    pose_pub = rospy.Publisher('waypoint_xyz', ME439WaypointXYZ, queue_size=10)
    path_pub = rospy.Publisher('path_complete', Bool, queue_size=10)

    rospy.init_node('data2Pose', anonymous=True)

    data2Pose = data2Pose()

    rospy.Subscriber('/imu_data', Quaternion , data2Pose.orientationCallback)
    rospy.Subscriber('/gyr_data', Vector3, data2Pose.velocityCallback)
    rospy.Subscriber('/joy', Int16MultiArray, data2Pose.joyCallback)

    rospy.spin()