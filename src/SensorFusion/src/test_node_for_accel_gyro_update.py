import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion,quaternion_multiply, quaternion_from_euler
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
roll = [0]
pitch = [0]
yaw = [0]

## CREATE A PUBLISHER FOR FUSED ORIENTATION
fused_pub = rospy.Publisher("/Fused_orientation_topic", SensorMsgStamped, queue_size = 10)

## CREATE A BROADCASTER TO PUBLISH CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()


#################code_update_for Gyro incorporation#########################
def got_accel_mag(msg_accel, msg_mag,msg_gyro):
    alpha = 0.1

    ###Change this to actual time####
    dt = 0.001 # random timestep
    #################################

    ## GET ROLL AND PITCH FROM THE ACCELEROMETER
    ax, ay, az = msg_accel.data
    gx, gy, gz = msg_gyro.data

    ## NORMALIZE THE ACCELEROMETER DATA
    norm_accel =  m.sqrt(ax**2 + ay**2 + az**2)
    ax = ax / norm_accel
    ay = ay / norm_accel
    az = az / norm_accel
    a_roll = m.atan2(ay, az)
    a_pitch = m.atan2(-ax, m.sqrt(ay**2 + az**2))
    a_yaw = m.tan2(a_roll,a_pitch) #### Check this #############

    ## Process GyroScope data
    current_gyro_quat =  quaternion_from_euler(gx*dt, gy*dt, gz*dt)
    q_check = quaternion_multiply(current_gyro_quat,q)
    g_roll,g_pitch,g_yaw = euler_from_quaternion(q_check)

    ##Combine Gyro and Accelerometer Roll pitch and yaw
    roll = (1-alpha)*g_roll + alpha*a_roll
    pitch = (1-alpha)*g_pitch + alpha*a_pitch
    yaw = (1-alpha)*g_yaw + alpha*a_yaw


    ## CONVERT TO quaternion_from_euler
    q = quaternion_from_euler(roll, pitch, yaw)

    ## LETS PUBLISH THIS TO THE BROADCASTER
    tf_br.sendTransform((1,1,1), (q[0],q[1],q[2],q[3]), Time.now(), 'phone', 'world')
#################code_update_for Gyro incorporation#########################



if __name__ == "__main__":
    try:
        mag_sub = message_filters.Subscriber("/Magneto_topic_stamped", SensorMsgStamped)
        accel_sub = message_filters.Subscriber("/Accel_topic_stamped", SensorMsgStamped)
        #############added__code###########################################################
        gyro_sub = message_filters.Subscriber("/Gyro_topic_stamped", SensorMsgStamped)
        #############added__code###########################################################
        combined_sub = message_filters.ApproximateTimeSynchronizer([accel_sub, mag_sub], queue_size = 5, slop = 0.1)
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
#
#
########################