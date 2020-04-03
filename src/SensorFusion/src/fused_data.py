#!/usr/bin/env python
import rospy
from sensor_fusion_pkg.msg import SensorMsg
from tf.transformations import euler_from_quaternion
from tf import TransformBroadcaster
from rospy import Time
import numpy as np
import math as m



## INITIALIZE THE NODE
rospy.init_node("Fused_data_node")

## GLOBAL VARIABLES
curtime = Time.now().secs + Time.now().nsecs * 10 ** (-9)
quat = [curtime,0,0,0,1]

## CREATE A PUBLISHER FOR FUSED ORIENTATION
fused_pub = rospy.Publisher("/Fused_orientation_topic", SensorMsg, queue_size = 10)

## CREATE BROADCASTER TO PUBLISH TO CORRECT FRAMES FOR VISUALIZATION
tf_br = TransformBroadcaster()

def fused_cb(msg):
    global quat
    timeStamp, wx, wy, wz = msg.data
    q0, q1, q2, q3, q4 = quat # q0 is time
    ## COMPUTE THE TRANSFORMATION MATRIX
    quatMat = 0.5 * np.array([[q4, -q3, q2],[q3, q4, -q1],[-q2, q1, q4], [-q1, -q2, -q3]])
    rates = np.array([[wx], [wy], [wz]])
    ## FIND THE dq VALUES
    dquat = quatMat.dot(rates)
    dq1 = float(dquat[0])
    dq2 = float(dquat[1])
    dq3 = float(dquat[2])
    dq4 = float(dquat[3])
    ## UPDATE THE QUATERNION
    quat[0] = Time.now().secs + Time.now().nsecs * 10 ** (-9)
    quat[1] = quat[1] + dq1 * (quat[0] - q0)
    quat[2] = quat[2] + dq2 * (quat[0] - q0)
    quat[3] = quat[3] + dq3 * (quat[0] - q0)
    quat[4] = quat[4] + dq4 * (quat[0] - q0)


    ## NORMALIZE QUATERNION
    norm = m.sqrt(quat[1]**2 + quat[2]**2 + quat[3]**2 + quat[4]**2)
    quat[1] = quat[1]/norm
    quat[2] = quat[2]/norm
    quat[3] = quat[3]/norm
    quat[4] = quat[4]/norm

    ## CONVERT TO EULER FOR PLOTTING
    (roll, pitch, yaw) = euler_from_quaternion([quat[1], quat[2], quat[3], quat[4]])

    ## PUBLISH MESSAGE AS WELL AS TRANSFORM
    fused_pub.publish([timeStamp, m.degrees(roll), m.degrees(pitch), m.degrees(yaw)])
    tf_br.sendTransform((1,1,1), (quat[1],quat[2],quat[3],quat[4]), Time.now(), 'phone_link', 'base_link')


if __name__ == "__main__":
    try:
        rospy.Subscriber("/Gyro_topic", SensorMsg, fused_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# """
# TASKS AND INFERENCES
# 1. GET THE GYRO DATA
# 2. INITIALIZE THE QUATERNION TO [0,0,0,1]
# 3. UPDATE USING THE MOBICOM EQUATION
# 4. CONVERT TO EULER FOR PLOTTING
# 5. ANALYZE THE PLOT USING RQT_PLOT
# 6. THERE IS THE USUAL DRIFT IN READINGS AS TIME PROGRESSES BUT VALUES ARE STABLE
# """
