#!/usr/bin/env python
# license removed for brevity

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


class mocapPose:
    def __init__(self):
        self.b = TransformBroadcaster()
        self.translation = (0.0, 0.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.rate = rospy.Rate(100)
        # Initialize the publisher for the marker
        self.pos = [0,0,0]
        self.mocap_eul_pub = rospy.Publisher('/mocap_eul', Vector3, queue_size=1)
        self.eul = Vector3()

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q0 = 1

    def mocapPoseCallback(self, msg):
        # "Store" message received.
        self.q1 = msg.pose.orientation.x
        self.q2 = msg.pose.orientation.y
        self.q3 = msg.pose.orientation.z
        self.q0 = msg.pose.orientation.w
        # Compute stuff.
        self.compute_stuff()
            

    def compute_stuff(self):
        
        quat = np.array([self.q1, self.q2, self.q3, self.q0])
        rotate = np.array([0,0.7071,0,0.7071])
        #rotate = np.array([0,0,0, 1])
        rotation = self.quaternion_multiply(rotate, quat)
        rotation = (-rotation[0], -rotation[2], -rotation[1], rotation[3]) #x,y,z,w
        quat = (rotation[3], rotation[0], rotation[1], rotation[2]) #w,x,y,z
        self.angles = quat2euler(quat)
        self.eul.x = self.angles[0]*180/np.pi
        self.eul.y = self.angles[1]*180/np.pi
        self.eul.z = self.angles[2]*180/np.pi
        self.mocap_eul_pub.publish(self.eul)

        # rospy.loginfo(np.linalg.norm(r_acc))
        self.b.sendTransform(self.pos, rotation,Time.now(), 'mocap', '/world')

    def quaternion_multiply(self,quaternion1, quaternion0):
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                    x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                    -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)

if __name__ == '__main__':
    rospy.init_node('mocapPose', anonymous=True)

    mocapPose = mocapPose()

    rospy.Subscriber('/vrpn_client_node/RigidBody/pose', PoseStamped , mocapPose.mocapPoseCallback)
    rospy.spin()