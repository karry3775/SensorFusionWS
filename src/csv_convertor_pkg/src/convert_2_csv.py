#!/usr/bin/env python
import rospy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

## create a node
rospy.init_node("csv_convertor_node")
# gps_file = open("gps_data.csv", "w")
# gps_file.write("time, x ,y, z, roll, pitch, yaw")
imu_file = open("imu_data.csv", "w")
imu_file.write("time, roll, pitch, yaw, wx, wy, wz, ax, ay, az\n")

def gps_cb(data):
    global gps_file
    time_val = data.header.stamp.secs
    position = data.pose.pose.position
    x = position.x
    y = position.y
    z = position.z
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    gps_file.write("{},{},{},{},{},{},{} \n".format(time_val,x,y,z,roll,pitch,yaw))

def imu_cb(data):
    global imu_file
    time_val = data.header.stamp.secs
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    wx = data.angular_velocity.x
    wy = data.angular_velocity.y
    wz = data.angular_velocity.z
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    imu_file.write("{},{},{},{},{},{},{},{},{},{}\n".format(time_val,roll, pitch, yaw,wx,wy,wz,ax,ay,az))

if __name__ == "__main__":
    try:
        # rospy.Subscriber("/navsat/odom", Odometry, gps_cb)
        rospy.Subscriber("/imu/data", Imu, imu_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        # gps_file.close()
        imu_file.close()
