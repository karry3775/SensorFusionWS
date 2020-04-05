#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import message_filters

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

    # lets publish this transform
    tf_br.sendTransform((0, 0, 0.5), (q[0],q[1],q[2],q[3]), Time.now(), "base_link", "world")

def fusion_cb(msg_gyro, msg_accel, msg_mag):
    global q, prev_time, P, INI_SET

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
        INI_SET = True
        # there is no gyro measurement yet
        rollF = rollA
        pitchF = pitchA
        yawF = yawM
        q = quaternion_from_euler(rollF, pitchF, yawF)
        P = R # and thus the initial noise would be equal to noise in measurement

    print("[FUSION_CB] rollF : {}, pitchF : {}, yawF : {}, P: \n{}".format(m.degrees(rollF), m.degrees(pitchF), m.degrees(yawF), P))

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
