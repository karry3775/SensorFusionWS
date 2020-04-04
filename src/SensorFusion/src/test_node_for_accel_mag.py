#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import message_filters # for time synchronization
## INITIALIZE THE NODE
rospy.init_node("RPY_accel_mag_node")

## GLOBAL VARIABLES
curtime = Time.now().secs + Time.now().nsecs * 10 ** (-9)
orientation = []
DEBUG = True

## CREATE A PUBLISHER FOR FUSED ORIENTATION
fused_pub = rospy.Publisher("/Fused_orientation_topic", SensorMsgStamped, queue_size = 10)

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()

def got_accel_mag(msg_accel, msg_mag):
    ## GET ROLL AND PITCH FROM THE ACCELEROMETER
    ax, ay, az = msg_accel.data
    mx, my, mz = msg_mag.data

    if DEBUG:
        print("raw acceleration values ax : {}, ay : {}, az : {}".format(ax, ay, az))
        print("raw magnetometer values mx : {}, my : {}, mz : {}".format(mx, my, mz))

    ## NORMALIZE THE ACCELEROMETER DATA
    norm_accel =  m.sqrt(ax**2 + ay**2 + az**2)
    ax = ax / norm_accel
    ay = ay / norm_accel
    az = az / norm_accel

    # ## NORMALIZE THE MAGNETOMETER DATA
    norm_mag =  m.sqrt(mx**2 + my**2 + mz**2)
    mx = mx / norm_mag
    my = my / norm_mag
    mz = mz / norm_mag

    if DEBUG:
        print("normalized acceleration values ax : {}, ay : {}, az : {}".format(ax, ay, az))
        print("normalized magnetometer values mx : {}, my : {}, mz : {}".format(mx, my, mz))

    ## GET THE ROLL AND PITCH
    rollA = m.atan2(ay, az)
    pitchA = m.atan2(-ax, m.sqrt(ay**2 + az**2))

    ## COMPENSATE FOR YAW USING MAGNETOMETER
    Mx = mx * m.cos(pitchA) + mz * m.sin(pitchA)
    My = mx * m.sin(rollA) * m.sin(pitchA) + my * m.cos(rollA) - mz * m.sin(rollA) * m.cos(pitchA)

    yawM = m.atan2(-My, Mx)

    if DEBUG:
        print("rollA : {}, pitchA : {}, yawM : {}".format(m.degrees(rollA), m.degrees(pitchA), m.degrees(yawM)))
        print("###############################################################################")
        print("                                                                               ")

    ## CONVERT TO quaternion_from_euler
    q = quaternion_from_euler(rollA, pitchA, yawM)

    ## LETS PUBLISH THIS TO THE BROADCASTER
    tf_br.sendTransform((0.0,0.0,0.5), (q[0],q[1],q[2],q[3]), Time.now(), 'base_link', 'world')

if __name__ == "__main__":
    try:
        mag_sub = message_filters.Subscriber("/Magneto_topic_stamped", SensorMsgStamped)
        accel_sub = message_filters.Subscriber("/Accel_topic_stamped", SensorMsgStamped)
        combined_sub = message_filters.ApproximateTimeSynchronizer([accel_sub, mag_sub], queue_size = 5, slop = 0.05)
        combined_sub.registerCallback(got_accel_mag)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


########################
# TO UNDERSTAND FUSION BETWEEN ACCELEROMETER AND MAGNETOMETER
# Author : Kartik Prakash
# Date   : 7/Mar/2020
# STEPS:
# 1. Get the raw accel values and the magnetometer values
# 2. Try combining resultant accel and magnetometer to figure out RPY
# 3. Figure out why yaw values are not accurate.
#
########################
