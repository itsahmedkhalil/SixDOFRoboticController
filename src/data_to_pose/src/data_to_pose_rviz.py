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



class data2Pose:
    """
    Joystick and IMU data converted into pose for rviz visualization
    """
    def __init__(self):
        self.b = TransformBroadcaster()
        self.translation = (0.0, 0.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.rate = rospy.Rate(500)
        # Initialize the publisher for the marker (you can use it for the marker game!)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker initial position
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 1
        self.marker.id = 0
        # Set the scale of the marker
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.pos = [0,0,0]

        self.dt = 0.01

        self.angVelx = 0
        self.angVely = 0
        self.angVelz = 0

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q0 = 1

    def orientationCallback(self, msg):
        # "Recieve" IMU quaternion message.
        self.q1 = msg.x
        self.q2 = msg.y
        self.q3 = msg.z
        self.q0 = msg.w
        # Compute stuff.
        self.compute_stuff()

    def velocityCallback(self, msg):
        # "Recieve" IMU angular velocity message.
        self.angVelx = msg.x
        self.angVely = msg.y
        self.angVelz = msg.z

        # Compute stuff.
        self.compute_stuff()
    
    def joyCallback(self, msg):
        # "Recieve" joystick message and convert it into desired "velocity" values.
        joyX = msg.data[0]
        joyY = msg.data[1]
        joyZ = msg.data[2]
        self.joy = [joyY,joyX,joyZ]
        vel = [0,0,0]
        for i in range(2):
            if abs(self.joy[i]) < 15:
                self.joy[i] = 0
            else:
                self.joy[i] = self.joy[i]*0.001
            

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        # Rotate the velocity data from the joystick by the rotation of the IMU
        # && publish the tf 
        rotation = (self.q1, self.q2, self.q3, self.q0)
        r = R.from_quat(np.array([self.q1, self.q2, self.q3, self.q0]))
        r_pos = r.apply(np.array(self.joy))
        self.pos+=r_pos*self.dt
        self.marker_pub.publish(self.marker)
        self.b.sendTransform(self.pos, rotation,Time.now(), 'imu', '/world')


if __name__ == '__main__':
    rospy.init_node('data2Pose', anonymous=True)

    data2Pose = data2Pose()

    rospy.Subscriber('/imu_data', Quaternion , data2Pose.orientationCallback)
    rospy.Subscriber('/gyr_data', Vector3, data2Pose.velocityCallback)
    rospy.Subscriber('/joy', Int16MultiArray, data2Pose.joyCallback)

    rospy.spin()