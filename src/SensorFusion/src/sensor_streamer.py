#!/usr/bin/env python

"""
This code takes Android data using PhonePi App and streams it to accel_topic, magneto_topic, gyro_topic
"""
import rospy
import sys
from flask import Flask
from flask_sockets import Sockets
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from sensor_fusion_pkg.msg import SensorMsg, SensorMsgStamped
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

from tf import TransformBroadcaster
from rospy import Time

"""
GLobal data that can be shared between socket communication and ros publishers
"""
gyro_data = []
accel_data = []
magneto_data = []
orient_data = []
geoloc_data = []
# lets create our publishers
rospy.init_node("sensor_streamer_node",disable_signals=True)

## STAMPED PUBLISHERS
gyro_pub_stamped = rospy.Publisher("/Gyro_topic_stamped", SensorMsgStamped, queue_size = 10)
accel_pub_stamped = rospy.Publisher("/Accel_topic_stamped", SensorMsgStamped, queue_size = 10)
magneto_pub_stamped = rospy.Publisher("/Magneto_topic_stamped", SensorMsgStamped, queue_size = 10)
orient_pub_stamped = rospy.Publisher("/Orientation_topic_stamped", SensorMsgStamped, queue_size = 10)
geoloc_pub_stamped = rospy.Publisher("/Geolocation_topic_stamped", SensorMsgStamped, queue_size = 10)

"""
Code snippet from PhonePi.py
"""
app = Flask(__name__)
sockets = Sockets(app)

@sockets.route('/accelerometer')
def echo_socket(ws):
	f=open("accelerometer.txt","a")
	while True:
		message = ws.receive()
		accel_data = message.split(',')
		accel_data = [float(data) for data in accel_data]

		stamped_msg = SensorMsgStamped()
		stamped_msg.data = accel_data
		stamped_msg.header.stamp.secs = Time.now().secs
		stamped_msg.header.stamp.nsecs = Time.now().nsecs

		accel_pub_stamped.publish(stamped_msg)

		print("[INFO:] Accelerometer{}".format(accel_data))
        ws.send(message)
        print>>f,message

	f.close()


@sockets.route('/gyroscope')
def echo_socket(ws):
	f=open("gyroscope.txt","a")
	while True:
		message = ws.receive()
		gyro_data = message.split(',')
		gyro_data = [float(data) for data in gyro_data]

		stamped_msg = SensorMsgStamped()
		stamped_msg.data = gyro_data
		stamped_msg.header.stamp.secs = Time.now().secs
		stamped_msg.header.stamp.nsecs = Time.now().nsecs

		gyro_pub_stamped.publish(stamped_msg)

		print("[INFO:] Gyroscope{}".format(gyro_data))
        ws.send(message)
        print>>f,message

	f.close()

@sockets.route('/magnetometer')
def echo_socket(ws):
	global magneto_data, magneto_pub
	f=open("magnetometer.txt","a")
	while True:
		message = ws.receive()
		magneto_data = message.split(',')
		magneto_data = [float(data) for data in magneto_data]

		stamped_msg = SensorMsgStamped()
		stamped_msg.data = magneto_data
		stamped_msg.header.stamp.secs = Time.now().secs
		stamped_msg.header.stamp.nsecs = Time.now().nsecs

		magneto_pub_stamped.publish(stamped_msg)

		print("[INFO:] Magnetometer{}".format(magneto_data))
        ws.send(message)
        print>>f,message

	f.close()

@sockets.route('/orientation')
def echo_socket(ws):
	b = TransformBroadcaster()

	f=open("orientation.txt","a")
	while True:
		message = ws.receive()
		orient_data = message.split(',')
		orient_data = [float(data) for data in orient_data]

		stamped_msg = SensorMsgStamped()
		stamped_msg.data = orient_data
		stamped_msg.header.stamp.secs = Time.now().secs
		stamped_msg.header.stamp.nsecs = Time.now().nsecs

		orient_pub_stamped.publish(stamped_msg)

		### Publish to Pose topic for visualization ###
		q = quaternion_from_euler(orient_data[1], orient_data[2], orient_data[3])
		pose_msg = Pose()
		pose_msg.orientation.x = q[0]
		pose_msg.orientation.y = q[1]
		pose_msg.orientation.z = q[2]
		pose_msg.orientation.w = q[3]
		pose_pub.publish(pose_msg)

		b.sendTransform((1,1,1), (q[0],q[1],q[2],q[3]), Time.now(), 'child_link', 'base_link')
		### END HERE ###
		print("[INFO:] Orientation{}".format(orient_data))
        ws.send(message)
        print>>f,message

	f.close()

@sockets.route('/geolocation')
def echo_socket(ws):
	global geoloc_data, geoloc_pub
	f=open("geolocation.txt","a")
	while True:
		message = ws.receive()
		geoloc_data = message.split(',')
		geoloc_data = [float(data) for data in geoloc_data]

		stamped_msg = SensorMsgStamped()
		stamped_msg.data = geoloc_data
		stamped_msg.header.stamp.secs = Time.now().secs
		stamped_msg.header.stamp.nsecs = Time.now().nsecs

		geoloc_pub_stamped.publish(stamped_msg)

		print("[INFO:] Geolocation{}".format(geoloc_data))
        ws.send(message)
        print>>f,message

	f.close()

@app.route('/')
def hello():
	return 'Hello World!'

if __name__ == "__main__":
	try:
		server = pywsgi.WSGIServer(('0.0.0.0', 5000), app, handler_class=WebSocketHandler)
		server.serve_forever()
	except rospy.ROSInterruptException:
		server.close()
