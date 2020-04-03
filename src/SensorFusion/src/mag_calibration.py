#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsgStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m
import matplotlib.pyplot as plt


## INITIALIZE NODE
rospy.init_node("mag_calib_node")

## GLOBAL VARIABLES
X = []
Y = []
Z = []

def mag_cb(msg):
    global X, Y, Z
    mx, my, mz = msg.data
    X.append(mx)
    Y.append(my)
    Z.append(mz)

if __name__ == "__main__":
    try:
        rospy.Subscriber("/Magneto_topic_stamped", SensorMsgStamped, mag_cb)
        rospy.spin()

        # LETS PLOT THE GRAPH
        plt.plot(X, Y, 'ro', label = "XY")
        plt.plot(X, Z, 'bo', label = "XZ")
        plt.plot(Y, Z, 'co', label = "YZ")
        plt.axis('equal')
        plt.legend()
        plt.show()

    except rospy.ROSInterruptException:
        pass
