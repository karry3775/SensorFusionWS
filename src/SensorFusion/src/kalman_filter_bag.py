#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_matrix
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import message_filters

import matplotlib.pyplot as plt

## INITIALIZE NODE
rospy.init_node("Kalman_filter_node")
# create RPY publisher for fused data from kalman filtering
rpyKF_pub_stamped = rospy.Publisher("/RPYKF_topic_stamped", SensorMsgStamped, queue_size = 10)

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()

## GLOBAL VARIABLES, F for fused
q = None
INI_SET = False
prev_time = Time.now().secs + Time.now().nsecs * 10 ** (-9)

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

## KALMAN FILTER VARIABLES
# state covariance (initial is implicit)
P = np.array([[100, 0, 0],
              [0, 100, 0],
              [0, 0, 100]])
# process noise (assumed randomly , can be refined using statistical analysis of gyro)
Q = np.array([[0.01, 0, 0],
              [0, 0.01, 0],
              [0, 0, 0.01]])

# measurement noise (assumed randomly, can be refined using statistical analysis of accel and magnetometer)
R = np.array([[0.1, 0, 0],
              [0, 0.1, 0],
              [0, 0, 0.1]])

dt = 0.1

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def integrateTillT(q, dt, time_elapsed, wx, wy, wz, P):
    if time_elapsed <= dt:
        dt = time_elapsed

    # integrate using euler method
    ini_time = dt
    while(ini_time <= time_elapsed):
        # make the state prediction
        current_gyro_quat = quaternion_from_euler(wx * dt, wy * dt, wz * dt)
        q = quaternion_multiply(current_gyro_quat, q)
        # make the covariance prediction
        P += Q
        ini_time += dt

    # we will run into complications if time_elapsed is not evenly divided by dt
    if ((ini_time - time_elapsed) - dt > 0.001):
        # we will need to integrate once more
        del_t = time_elapsed - (ini_time - dt)
        current_gyro_quat = quaternion_from_euler(wx * del_t, wy * del_t, wz * del_t)
        q = quaternion_multiply(current_gyro_quat, q)
        P += Q

    return q, P

def gyro_cb(msg_gyro):
    global q, prev_time, P, INI_SET
    if not INI_SET:
        # initial state is not set
        return

    # extract data
    wx, wy, wz = msg_gyro.data

    # we need to calculate time_elapsed
    secs = Time.now().secs
    nsecs = Time.now().nsecs
    cur_time = secs + nsecs * 10 ** (-9)
    time_elapsed = cur_time - prev_time
    prev_time = cur_time

    # integrate using eulers integration method to find the estimates and covariance
    # prediction
    q, P = integrateTillT(q, dt, time_elapsed, wx, wy, wz, P)

    rollF, pitchF, yawF = euler_from_quaternion(q)
    print("[GYRO_CB] rollF : {}, pitchF : {}, yawF : {}, P: \n{}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF), P))

    # publish to topic
    stamped_msg = SensorMsgStamped()
    stamped_msg.data = [rollF, pitchF, yawF]
    stamped_msg.header.stamp.secs = secs
    stamped_msg.header.stamp.nsecs = nsecs

    rpyKF_pub_stamped.publish(stamped_msg)



def fusion_cb(msg_gyro, msg_accel, msg_mag):
    global q, prev_time, P, INI_SET, KF_roll, KF_pitch, KF_yaw, KF_time, roll_correction, pitch_correcton, yaw_correction

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

    ## rollA, pitchA, yawM are the measurements

    ## PREDICTION IF NEEDED
    if not SKIP_GYRO:
        # carry out prediction step
        # we need to calculate time_elapsed
        secs = Time.now().secs
        nsecs = Time.now().nsecs
        cur_time = secs + nsecs * 10 ** (-9)
        time_elapsed = cur_time - prev_time
        prev_time = cur_time

        # integrate using eulers integration method to find the estimates and covariance
        q, P = integrateTillT(q, dt, time_elapsed, wx, wy, wz, P)

        # extract angles
        rollF, pitchF, yawF = euler_from_quaternion(q)

        # compute kalman gain
        K = np.dot(P, np.linalg.inv(P + R))

        # carry out correction step
        meas = np.array([[rollA],[pitchA],[yawM]])
        state = np.array([[rollF],[pitchF],[yawF]])

        state = state + np.dot(K, meas - state)
        state = [float(val) for val in state]
        rollF, pitchF, yawF = state

        # update the quaternion
        q = quaternion_from_euler(rollF, pitchF, yawF)

        # update covariance
        P = np.dot((np.eye(3) - K), P)

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
        # there is no gyro measurement yet
        rollF = rollA
        pitchF = pitchA
        yawF = yawM
        q = quaternion_from_euler(rollF, pitchF, yawF)
        P = R # and thus the initial noise would be equal to noise in measurement

    print("[FUSION_CB] rollF : {}, pitchF : {}, yawF : {}, P: \n{}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF), P))

    # publish to topic
    stamped_msg = SensorMsgStamped()
    stamped_msg.data = [rollF, pitchF, yawF]
    stamped_msg.header.stamp.secs = Time.now().secs
    stamped_msg.header.stamp.nsecs = Time.now().nsecs

    rpyKF_pub_stamped.publish(stamped_msg)

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
