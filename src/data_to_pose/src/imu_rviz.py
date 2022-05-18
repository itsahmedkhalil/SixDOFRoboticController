#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from time import time
from geometry_msgs.msg import Vector3, Quaternion
from tf import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
from rospy import Time
from transforms3d.euler import quat2euler



class data2Pose:
    def __init__(self):
        self.b = TransformBroadcaster()
        self.translation = (0.0, 0.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.rate = rospy.Rate(100)
        # Initialize the publisher for the marker
        self.pos = [0,0,0]
        self.imu_eul_pub = rospy.Publisher('/imu_eul', Vector3, queue_size=1)
        self.eul = Vector3()

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q0 = 1


    def orientationCallback(self, msg):
        # "Store" message received.
        self.q1 = msg.x
        self.q2 = msg.y
        self.q3 = msg.z
        self.q0 = msg.w
        # Compute stuff.
        self.compute_stuff()
        
    def compute_stuff(self):
        
        rotation = (self.q1, self.q2, self.q3, self.q0)
        quat = (self.q0, self.q1, self.q2, self.q3)
        self.angles = quat2euler(quat)
        self.eul.x = self.angles[0]*180/np.pi
        self.eul.y = self.angles[1]*180/np.pi
        self.eul.z = self.angles[2]*180/np.pi
        self.imu_eul_pub.publish(self.eul)

        # rospy.loginfo(np.linalg.norm(r_acc))
        self.b.sendTransform(self.pos, rotation,Time.now(), 'imu', '/world')


if __name__ == '__main__':
    rospy.init_node('data2Pose', anonymous=True)

    data2Pose = data2Pose()
    rospy.Subscriber('/imu_data', Quaternion , data2Pose.orientationCallback)
    rospy.spin()