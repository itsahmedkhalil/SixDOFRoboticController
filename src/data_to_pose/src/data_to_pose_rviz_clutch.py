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
    def __init__(self):
        self.b = TransformBroadcaster()
        self.translation = (0.0, 0.0, 0.0)
        self.rotation = np.array([0.,0.,0.,1.])
        self.current_rot_offset = np.array([0.,0.,0.,1.])
        self.current_rot_offset_inv = np.array([0.,0.,0.,1.])
        self.rate = rospy.Rate(500)
        # Initialize the publisher for the marker
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
        joyY = msg.data[1]
        joyZ = msg.data[2]
        clutch = msg.data[3]
        if clutch == 1:
            self.clutch_is_pressed = True
        else:
            self.clutch_is_pressed = False
        self.joy = [joyX,-joyY,joyZ]
        vel = [0,0,0]
        for i in range(2):
            if abs(self.joy[i]) < 15:
                self.joy[i] = 0
            else:
                self.joy[i] = self.joy[i]*0.001
            

        # Compute stuff.
        self.compute_stuff()

    def compute_stuff(self):
        if not self.clutch_is_pressed:
            self.rotation_imu = np.array([self.q1, self.q2, self.q3, self.q0])
            print(self.current_rot_offset_inv)
            self.rotation = self.quaternion_multiply(self.current_rot_offset_inv, self.rotation_imu)
            #r = R.from_quat(np.array([self.q1, self.q2, self.q3, self.q0]))
            r = R.from_quat(self.rotation)
            rotation_tf = (self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3]) 


        else:
            self.current_rot_offset = self.rotation
            rot_inv = R.from_quat(self.current_rot_offset).inv()   #rotation matrix of global wrt initial frame
            self.current_rot_offset_inv  = rot_inv.as_quat()
            r = R.from_quat(self.current_rot_offset)
            rotation_tf = (self.current_rot_offset[0], self.current_rot_offset[1], self.current_rot_offset[2], self.current_rot_offset[3])
        r_pos = r.apply(np.array(self.joy))
        self.pos+=r_pos*self.dt
        self.marker_pub.publish(self.marker)

        self.b.sendTransform(self.pos, rotation_tf,Time.now(), 'imu', '/world')
    
    def quaternion_multiply(self,quaternion1, quaternion0):
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)


if __name__ == '__main__':
    rospy.init_node('data2Pose', anonymous=True)

    data2Pose = data2Pose()

    rospy.Subscriber('/imu_data', Quaternion , data2Pose.orientationCallback)
    rospy.Subscriber('/gyr_data', Vector3, data2Pose.velocityCallback)
    rospy.Subscriber('/joy', Int16MultiArray, data2Pose.joyCallback)

    rospy.spin()