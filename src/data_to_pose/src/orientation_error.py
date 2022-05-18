#!/usr/bin/env python
# license removed for brevity

from builtins import abs
import rospy
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from time import time
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from tf import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
from rospy import Time
from transforms3d.euler import quat2euler


class OrientationErrorNode:
    def __init__(self):
        self.rate = rospy.Rate(100)
        # Initialize the publisher for the marker
        self.error_eul_pub = rospy.Publisher('/orientation_error', Vector3, queue_size=1)
        self.error = Vector3()
        self.mocap_recieved = False
        self.imu_recieved = False

    def mocapEulerCallback(self, msg):
        # "Store" message received.
        self.mocap_eul_x = msg.x
        self.mocap_eul_y = msg.y
        self.mocap_eul_z = msg.z
        self.mocap_recieved = True
        # Compute stuff.
        self.compute_error()
    
    def imuEulerCallback(self, msg):
        # "Store" message received.
        self.imu_eul_x = msg.x
        self.imu_eul_y = msg.y
        self.imu_eul_z = msg.z
        self.imu_recieved = True
        # Compute stuff.
        self.compute_error()
            

    def compute_error(self):
        # Compute the error between the IMU and the Mocap orientation
        if self.mocap_recieved and self.imu_recieved:
            self.error.x = abs(self.imu_eul_x - self.mocap_eul_x)
            self.error.y = abs(self.imu_eul_y - self.mocap_eul_y)
            self.error.z = abs(self.imu_eul_z - self.mocap_eul_z)
            self.error_eul_pub.publish(self.error)
        else:
            pass
        #print(self.mocap_recieved,self.imu_recieved)
 


if __name__ == '__main__':
    rospy.init_node('OrientationErrorNode', anonymous=True)

    OrientationErrorNode = OrientationErrorNode()

    rospy.Subscriber('/mocap_eul', Vector3 , OrientationErrorNode.mocapEulerCallback)
    rospy.Subscriber('/imu_eul', Vector3 , OrientationErrorNode.imuEulerCallback)


    rospy.spin()