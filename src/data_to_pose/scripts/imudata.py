#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import os
from time import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rospy import Time
from scipy.spatial.transform import Rotation as R
import random
from geometry_msgs.msg import Quaternion, Twist, Vector3
import sys
sys.path.append('/home/mohamed/openzen/build')
# sys.path.append('/home/ahmedkhalil/openzen')
import openzen


def main(): 
    imuPub = rospy.Publisher("/imu_data", Quaternion, queue_size=2)
    angVelPub = rospy.Publisher("/gyr_data", Vector3, queue_size=2)
    angVel = Vector3()
    rospy.init_node("imu_publisher")
    rate = rospy.Rate(400)
    imu_angle = Quaternion()
    imu_angle.x = 0
    imu_angle.y = 0
    imu_angle.z = 0
    imu_angle.w = 1
    openzen.set_log_level(openzen.ZenLogLevel.Warning)

    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializing OpenZen library")
        sys.exit(1)

    error = client.list_sensors_async()

    # check for events
    sensor_desc_connect = None
    while True:
        zenEvent = client.wait_for_next_event()

        if zenEvent.event_type == openzen.ZenEventType.SensorFound:
            print ("Found sensor {} on IoType {}".format( zenEvent.data.sensor_found.name,
                zenEvent.data.sensor_found.io_type))
            if sensor_desc_connect is None:
                sensor_desc_connect = zenEvent.data.sensor_found

        if zenEvent.event_type == openzen.ZenEventType.SensorListingProgress:
            lst_data = zenEvent.data.sensor_listing_progress
            print ("Sensor listing progress: {} %".format(lst_data.progress * 100))
            if lst_data.complete > 0:
                break
    print ("Sensor Listing complete")

    if sensor_desc_connect is None:
        print("No sensors found")
        sys.exit(1)

    # connect to the first sensor found
    error, sensor = client.obtain_sensor(sensor_desc_connect)

    # or connect to a sensor by name
    #error, sensor = client.obtain_sensor_by_name("LinuxDevice", "LPMSCU2000003")

    if not error == openzen.ZenSensorInitError.NoError:
        print ("Error connecting to sensor")
        sys.exit(1)

    print ("Connected to sensor !")

    imu = sensor.get_any_component_of_type(openzen.component_type_imu)
    if imu is None:
        print ("No IMU found")
        sys.exit(1)

    ## read bool property
    error, is_streaming = imu.get_bool_property(openzen.ZenImuProperty.StreamData)
    if not error == openzen.ZenError.NoError:
        print ("Can't load streaming settings")
        sys.exit(1)

    print ("Sensor is streaming data: {}".format(is_streaming))

    while not rospy.is_shutdown():
        try:
            
            zenEvent = client.wait_for_next_event()

            # check if its an IMU sample event and if it
            # comes from our IMU and sensor component
            if zenEvent.event_type == openzen.ZenEventType.ImuData and \
                zenEvent.sensor == imu.sensor and \
                zenEvent.component.handle == imu.component.handle:

                imu_data = zenEvent.data.imu_data

            quat = np.array(imu_data.q)
            gyr = np.array(imu_data.g)
        
            imu_angle.w = quat[0]
            imu_angle.x = -quat[1]
            imu_angle.y = -quat[2]
            imu_angle.z = -quat[3]

            r = R.from_quat([-quat[1],-quat[2],-quat[3],quat[0]])
            print(r)
            angVel.x = gyr[0]*np.pi/180.0
            angVel.y = gyr[1]*np.pi/180.0
            angVel.z = gyr[2]*np.pi/180.0
            
            imuPub.publish(imu_angle)
            angVelPub.publish(angVel)

        except rospy.ROSInterruptException:
            pass
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass