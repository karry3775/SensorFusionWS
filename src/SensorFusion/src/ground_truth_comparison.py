#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import math as m
import matplotlib.pyplot as plt
import time

## Initialize the node
rospy.init_node("Ground_truth_comparison_node")
## Global variables
# Kalman Filter
kf_yaw = []
kf_roll = []
kf_pitch = []
kf_time = []

# Complementary Filter
cf_yaw = []
cf_roll = []
cf_pitch = []
cf_time = []

# Gyro only
g_only_yaw = []
g_only_roll = []
g_only_pitch = []
g_only_time = []

start_time = time.time()

def kf_cb(msg):
    global kf_yaw, kf_pitch, kf_roll
    roll, pitch, yaw = msg.data
    # convert to degrees and append
    kf_yaw.append(m.degrees(yaw))
    kf_pitch.append(m.degrees(pitch))
    kf_roll.append(m.degrees(roll))
    kf_time.append(time.time() - start_time)

def cf_cb(msg):
    global cf_yaw, cf_pitch, cf_roll
    roll, pitch, yaw = msg.data
    # convert to degrees and append
    cf_yaw.append(m.degrees(yaw))
    cf_pitch.append(m.degrees(pitch))
    cf_roll.append(m.degrees(roll))
    cf_time.append(time.time() - start_time)

def g_only_cb(msg):
    global g_only_yaw, g_only_pitch, g_only_roll
    roll, pitch, yaw = msg.data
    # convert to degrees and append
    g_only_yaw.append(m.degrees(yaw))
    g_only_pitch.append(m.degrees(pitch))
    g_only_roll.append(m.degrees(roll))
    g_only_time.append(time.time() - start_time)

if __name__ == "__main__":
    try:
        kf_sub = rospy.Subscriber("/RPYKF_topic_stamped", SensorMsgStamped, kf_cb)
        g_only_sub = rospy.Subscriber("/RPYG_topic_stamped", SensorMsgStamped, g_only_cb)
        cf_sub = rospy.Subscriber("/RPYCF_topic_stamped", SensorMsgStamped, cf_cb)
        rospy.spin()
        # plot
        fig, axs = plt.subplots(3)
        axs[0].set_title("YAW")
        axs[0].plot(kf_time, kf_yaw, 'r-', label = "kf")
        axs[0].plot(cf_time, cf_yaw, 'b-', label = "cf")
        axs[0].plot(g_only_time, g_only_yaw, 'm-', label = "gyro")
        axs[0].axhline(y = 0 , linestyle = "--")
        axs[0].axhline(y = 30 , linestyle = "--")
        axs[0].axhline(y = 60 , linestyle = "--")
        axs[0].axhline(y = 90 , linestyle = "--")
        axs[0].axhline(y = 120 , linestyle = "--")
        axs[0].axhline(y = 150 , linestyle = "--")

        axs[1].set_title("PITCH")
        axs[1].plot(kf_time, kf_pitch, 'r-', label = "kf")
        axs[1].plot(cf_time, cf_pitch, 'b-', label = "cf")
        axs[1].plot(g_only_time, g_only_pitch, 'm-', label = "gyro")
        axs[1].axhline(y = 0 , linestyle = "--")
        axs[1].axhline(y = 30 , linestyle = "--")
        axs[1].axhline(y = 60 , linestyle = "--")
        axs[1].axhline(y = 90 , linestyle = "--")
        axs[1].axhline(y = 120 , linestyle = "--")
        axs[1].axhline(y = 150 , linestyle = "--")

        axs[2].set_title("ROLL")
        axs[2].plot(kf_time, kf_roll, 'r-', label = "kf")
        axs[2].plot(cf_time, cf_roll, 'b-', label = "cf")
        axs[2].plot(g_only_time, g_only_roll, 'm-', label = "gyro")
        axs[2].axhline(y = 0 , linestyle = "--")
        axs[2].axhline(y = 30 , linestyle = "--")
        axs[2].axhline(y = 60 , linestyle = "--")
        axs[2].axhline(y = 90 , linestyle = "--")
        axs[2].axhline(y = 120 , linestyle = "--")
        axs[2].axhline(y = 150 , linestyle = "--")

        axs[2].set_xlabel("time in seconds")
        axs[2].set_ylabel("angles in degrees")

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        fig.legend(lines, labels, loc = 'upper center')
        plt.show()

    except rospy.ROSInterruptException:
        pass
