#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import message_filters # for time synchronization
import matplotlib.pyplot as plt

## INITIALIZE NODE
rospy.init_node("Complimentary_filter_node")
# create RPY publisher for fused data from complimentary filtering
rpyCF_pub_stamped = rospy.Publisher("/RPYCF_topic_stamped", SensorMsgStamped, queue_size = 10)

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()

## GLOBAL VARIABLES
q = None
INI_SET = False
prev_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
g_wt =  0.75
am_wt = 1 - g_wt

# Additional variables for benchmarking
comp_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)
comp_time_gt = None
roll_correction = None
pitch_correcton = None
yaw_correction = None

# Kalman filter varibles
KF_roll = []
KF_pitch = []
KF_yaw = []
KF_time = []

# Ground truth variables
GT_roll = []
GT_pitch = []
GT_yaw = []
GT_time = []

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def gyro_cb(msg_gyro):
    global q, prev_time, INI_SET
    if not INI_SET:
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

    # process gyroscope data
    current_gyro_quat = quaternion_from_euler(wx * dt, wy * dt, wz * dt)
    q = quaternion_multiply(current_gyro_quat, q)
    rollF, pitchF, yawF = euler_from_quaternion(q)

    print("[GYRO_CB] rollF : {}, pitchF : {}, yawF : {}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF)))

    # publish to topic
    stamped_msg = SensorMsgStamped()
    stamped_msg.data = [rollF, pitchF, yawF]
    stamped_msg.header.stamp.secs = secs
    stamped_msg.header.stamp.nsecs = nsecs

    rpyCF_pub_stamped.publish(stamped_msg)

def fusion_cb(msg_gyro, msg_accel, msg_mag):
    global q, prev_time, INI_SET, KF_roll, KF_pitch, KF_yaw, KF_time, roll_correction, pitch_correcton, yaw_correction

    SKIP_GYRO = False
    if not INI_SET:
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

        # process gyroscope data
        current_gyro_quat = quaternion_from_euler(wx * dt, wy * dt, wz * dt)
        q = quaternion_multiply(current_gyro_quat, q)
        rollG, pitchG, yawG = euler_from_quaternion(q)

        ### COMPLIMENTARY FILTER ###
        rollF = rollG * g_wt + rollA * am_wt
        pitchF = pitchG * g_wt + pitchA * am_wt
        yawF = yawG * g_wt + yawM * am_wt

        # set back the quaternion
        q = quaternion_from_euler(rollF, pitchF, yawF)
    else:
        if comp_time_gt is not None:
            roll_gt = GT_roll[0]
            pitch_gt = GT_pitch[0]
            yaw_gt = GT_yaw[0]

            roll_ned = rollA
            pitch_ned = pitchA
            yaw_ned = yawM

            roll_correction = wrapToPi(roll_gt - roll_ned)
            pitch_correcton = wrapToPi(pitch_gt - pitch_ned)
            yaw_correction = wrapToPi(yaw_gt - yaw_ned)

        else:
            return
        INI_SET = True
        q = quaternion_from_euler(rollA, pitchA, yawM)

    rollF, pitchF, yawF =  euler_from_quaternion(q)
    print("[FUSION_CB] rollF : {}, pitchF : {}, yawF : {}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF)))

    # publish to topic
    stamped_msg = SensorMsgStamped()
    stamped_msg.data = [rollF, pitchF, yawF]
    stamped_msg.header.stamp.secs = Time.now().secs
    stamped_msg.header.stamp.nsecs = Time.now().nsecs

    rpyCF_pub_stamped.publish(stamped_msg)

    # get time
    t = Time.now().secs + Time.now().nsecs * 10 ** (-9) - comp_time

    # append to Global variables
    KF_roll.append(wrapToPi(rollF + roll_correction))
    KF_pitch.append(wrapToPi(pitchF + pitch_correcton))
    KF_yaw.append(wrapToPi(yawF + yaw_correction))
    KF_time.append(t)

def gt_rot_cb(msg):
    global GT_roll, GT_pitch, GT_yaw, GT_time, comp_time_gt
    if comp_time_gt is None:
        comp_time_gt = msg.header.stamp.to_sec()
    t = msg.header.stamp.to_sec() - comp_time_gt
    q = msg.quaternion
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    GT_time.append(t)
    GT_roll.append(roll)
    GT_pitch.append(pitch)
    GT_yaw.append(yaw)

if __name__ == "__main__":
    try:
        mag_sub = message_filters.Subscriber("/Magneto_topic_stamped", SensorMsgStamped)
        accel_sub = message_filters.Subscriber("/Accel_topic_stamped", SensorMsgStamped)
        gyro_sub = message_filters.Subscriber("/Gyro_topic_stamped", SensorMsgStamped)

        rpyG_sub = rospy.Subscriber("/Gyro_topic_stamped", SensorMsgStamped, gyro_cb)
        fusion_sub = message_filters.ApproximateTimeSynchronizer([gyro_sub, accel_sub, mag_sub], queue_size = 5, slop = 0.05)
        fusion_sub.registerCallback(fusion_cb)

        gt_rot_sub = rospy.Subscriber("/ee_rotation_stamped", QuaternionStamped, gt_rot_cb)
        rospy.spin()
        # plot
        fig, axs = plt.subplots(3)
        axs[0].set_title("ROLL")
        axs[0].plot(KF_time, KF_roll, "b-")
        axs[0].plot(GT_time, GT_roll, "r-")

        axs[1].set_title("PITCH")
        axs[1].plot(KF_time, KF_pitch, "b-")
        axs[1].plot(GT_time, GT_pitch, "r-")

        axs[2].set_title("YAW")
        axs[2].plot(KF_time, KF_yaw, "b-")
        axs[2].plot(GT_time, GT_yaw, "r-")

        plt.show()

    except rospy.ROSInterruptException:
        pass
