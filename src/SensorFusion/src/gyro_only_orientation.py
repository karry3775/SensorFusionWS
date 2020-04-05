#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m

## INITIALIZE THE NODE
rospy.init_node("Gyro_orientation_node")
# create RPY publisher for gyro
rpyG_pub_stamped = rospy.Publisher("/RPYG_topic_stamped", SensorMsgStamped, queue_size = 10)


## GLOBAL VARIABLES
prev_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
DEBUG = True

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()
q = quaternion_from_euler(0,0,0)

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def gyro_cb(msg_gyro):
    global rollG, pitchG, yawG, prev_time, q
    # extract data
    wx, wy, wz = msg_gyro.data

    # we need to calculate dt
    secs = Time.now().secs
    nsecs = Time.now().nsecs
    cur_time =  secs + nsecs * 10 ** (-9)
    dt = cur_time - prev_time
    prev_time = cur_time

    # process gyroscope data
    current_gyro_quat = quaternion_from_euler(wx * dt, wy * dt, wz * dt)
    q = quaternion_multiply(current_gyro_quat, q)
    rollG, pitchG, yawG = euler_from_quaternion(q)

    # lets publish this transform
    tf_br.sendTransform((0, 0, 0.5), (q[0],q[1],q[2],q[3]), Time.now(), "base_link", "world")

    # print the rollG, pitchG, yawG
    if DEBUG:
        print("rollG : {}, pitchG : {}, yawG : {}".format(m.degrees(rollG), m.degrees(pitchG), m.degrees(yawG)))

    # publish the roll pitch yaw topic
    stamped_msg = SensorMsgStamped()
    stamped_msg.data = [rollG, pitchG, yawG]
    stamped_msg.header.stamp.secs = secs
    stamped_msg.header.stamp.nsecs = nsecs

    rpyG_pub_stamped.publish(stamped_msg)

if __name__ == "__main__":
    try:
        gyro_sub = rospy.Subscriber("/Gyro_topic_stamped", SensorMsgStamped, gyro_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
