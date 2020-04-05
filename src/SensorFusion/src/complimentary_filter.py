#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import message_filters # for time synchronization

## INITIALIZE NODE
rospy.init_node("Complimentary_filter_node")
# create RPY publisher for fused data from complimentary filtering
rpyCF_pub_stamped = rospy.Publisher("/RPYCF_topic_stamped", SensorMsgStamped, queue_size = 10)

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()

## GLOBAL VARIABLES
rollF = None;
pitchF = None;
yawF = None;
prev_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
g_wt = 0.75
am_wt = 0.25


def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def gyro_cb(msg_gyro):
    global rollF, pitchF, yawF, prev_time
    if rollF == None:
        return
    # extract data
    wx, wy, wz = msg_gyro.data

    # we need to calculate dt
    secs = Time.now().secs
    nsecs = Time.now().nsecs
    cur_time = secs + nsecs * 10 ** (-9)
    dt = cur_time - prev_time
    prev_time = cur_time

    print("the time gap is: {}".format(dt))

    # calculate the roll pitch and yaw
    rollF = wrapToPi(rollF + wx * dt);
    pitchF = wrapToPi(pitchF + wy * dt);
    yawF = wrapToPi(yawF + wz * dt);

    print("[GYRO_CB] rollF : {}, pitchF : {}, yawF : {}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF)))

    ## SEND TRANSFORM
    # convert to quaternion_from_euler
    q = quaternion_from_euler(rollF, pitchF, yawF)

    # lets publish this transform
    tf_br.sendTransform((0, 0, 0.5), (q[0],q[1],q[2],q[3]), Time.now(), "base_link", "world")

def fusion_cb(msg_gyro, msg_accel, msg_mag):
    global rollF, pitchF, yawF, prev_time

    SKIP_GYRO = False
    if not rollF:
        SKIP_GYRO = True
    ## extract data
    ax, ay, az = msg_accel.data
    mx, my, mz = msg_mag.data
    wx, wy, wz = msg_gyro.data

    ### ACCELEROMETER AND MAGNETOMETER CALCULATIONS ###

    ## normalize accelerometer data
    norm_accel =  m.sqrt(ax**2 + ay**2 + az**2)
    ax = ax / norm_accel
    ay = ay / norm_accel
    az = az / norm_accel

    ## normalize the magnetometer data
    norm_mag =  m.sqrt(mx**2 + my**2 + mz**2)
    mx = mx / norm_mag
    my = my / norm_mag
    mz = mz / norm_mag

    ## get the roll and pitch
    rollA = m.atan2(ay, az)
    pitchA = m.atan2(-ax, m.sqrt(ay**2 + az**2))

    ## compensate for yaw using magnetometer
    Mx = mx * m.cos(pitchA) + mz * m.sin(pitchA)
    My = mx * m.sin(rollA) * m.sin(pitchA) + my * m.cos(rollA) - mz * m.sin(rollA) * m.cos(pitchA)

    yawM = m.atan2(-My, Mx)

    if not SKIP_GYRO:
        ### GYROSCOPE CALCULATIONS ###

        ## compute roll, pitch and yaw from gyro
        secs = Time.now().secs
        nsecs = Time.now().nsecs
        cur_time =  secs + nsecs * 10 ** (-9)
        dt = cur_time - prev_time
        prev_time = cur_time

        print("the time gap is: {}".format(dt))
        
        # calculate roll, pitch and yaw values
        rollG = wrapToPi(rollF + wx * dt);
        pitchG = wrapToPi(pitchF + wy * dt);
        yawG = wrapToPi(yawF + wz * dt);

        ### COMPLIMENTARY FILTER ###
        rollF = rollG * g_wt + rollA * am_wt
        pitchF = pitchG * g_wt + pitchA * am_wt
        yawF = yawG * g_wt + yawM * am_wt
    else:
        rollF = rollA
        pitchF = pitchA
        yawF = yawM

    print("[FUSION_CB] rollF : {}, pitchF : {}, yawF : {}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF)))

    ## SEND TRANSFORM
    # convert to quaternion_from_euler
    q = quaternion_from_euler(rollF, pitchF, yawF)

    # lets publish this transform
    tf_br.sendTransform((0, 0, 0.5), (q[0],q[1],q[2],q[3]), Time.now(), "base_link", "world")



if __name__ == "__main__":
    try:
        mag_sub = message_filters.Subscriber("/Magneto_topic_stamped", SensorMsgStamped)
        accel_sub = message_filters.Subscriber("/Accel_topic_stamped", SensorMsgStamped)
        gyro_sub = message_filters.Subscriber("/Gyro_topic_stamped", SensorMsgStamped)

        rpyG_sub = rospy.Subscriber("/Gyro_topic_stamped", SensorMsgStamped, gyro_cb)
        fusion_sub = message_filters.ApproximateTimeSynchronizer([gyro_sub, accel_sub, mag_sub], queue_size = 5, slop = 0.05)
        fusion_sub.registerCallback(fusion_cb)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
