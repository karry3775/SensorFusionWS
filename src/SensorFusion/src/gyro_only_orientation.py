#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m

## INITIALIZE THE NODE
rospy.init_node("Gyro_orientation_node")

## GLOBAL VARIABLES
prev_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
DEBUG = True

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()
rollG = 0;
pitchG = 0
yawG = 0


def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def gyro_cb(msg_gyro):
    global rollG, pitchG, yawG, prev_time
    # extract data
    wx, wy, wz = msg_gyro.data

    # we need to calculate dt
    cur_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
    dt = cur_time - prev_time
    prev_time = cur_time

    # calculate roll, pitch and yaw values
    rollG = wrapToPi(rollG + wx * dt);
    pitchG = wrapToPi(pitchG + wy * dt);
    yawG = wrapToPi(yawG + wz * dt);

    # convert to quaternion_from_euler
    q = quaternion_from_euler(rollG, pitchG, yawG)

    # lets publish this transform
    tf_br.sendTransform((0, 0, 0.5), (q[0],q[1],q[2],q[3]), Time.now(), "base_link", "world")

    # print the rollG, pitchG, yawG
    if DEBUG:
        print("rollG : {}, pitchG : {}, yawG : {}".format(m.degrees(rollG), m.degrees(pitchG), m.degrees(yawG)))

if __name__ == "__main__":
    try:
        gyro_sub = rospy.Subscriber("/Gyro_topic_stamped", SensorMsgStamped, gyro_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
