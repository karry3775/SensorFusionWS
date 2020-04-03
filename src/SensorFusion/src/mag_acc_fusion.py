#!/usr/bin/env python
import rospy
import sys
import numpy as np
import math
# from scipy.spatial.transform import Rotation as R
from sensor_fusion_pkg.msg import SensorMsg

class Mag_Fusion:

	def __init__(self):
		self.acc_sub = rospy.Subscriber('/Accel_topic', SensorMsg, self.acc_callback)
		self.mag_sub = rospy.Subscriber('/Magneto_topic', SensorMsg, self.mag_callback)
		self.mag_rpy_pub = rospy.Publisher('/Mag_rpy', SensorMsg, queue_size = 10)
		self.acc_vals = np.zeros(3)
		self.mag_vals = np.zeros(3)
		self.q0 = np.zeros(8)
		self.q1 = np.zeros(8)
		self.q2 = np.zeros(8)
		self.q3 = np.zeros(8)
		self.q_min = np.zeros(4)


	def acc_callback(self, data):
		self.acc_vals[0] = data.data[0]
		self.acc_vals[1] = data.data[1]  #NEED TO NORMALIZE
		self.acc_vals[2] = data.data[2]

	def mag_callback(self, data):
		self.mag_vals[0] = data.data[0]
		self.mag_vals[1] = data.data[1]  #NEED TO NORMALIZE OBSERVATION VECTORS
		self.mag_vals[2] = data.data[2]
		alpha = np.dot(self.acc_vals, self.mag_vals)
		mD = alpha
		mN = np.sqrt(1 - alpha**2)   #Used for quaternion selector
		X3 = (4 - 4*alpha**2)*(1 + self.acc_vals[2])**2
		X4 = (-self.acc_vals[0]**2 + self.acc_vals[1]**2 + (1 + self.acc_vals[2])**2)*self.mag_vals[0] - \
			2*self.acc_vals[0]*(self.acc_vals[1]*self.mag_vals[1] + self.mag_vals[2] + self.acc_vals[2]*self.mag_vals[2])

		print('x3 = ', X3)
		print('x4 = ', X4)

		self.q3[0] = -0.5*np.sqrt((1+self.acc_vals[2])*(1+(np.abs(X4)/np.sqrt(X3))))
		self.q3[1] = -0.5*np.sqrt((1+self.acc_vals[2])*(1-(np.abs(X4)/np.sqrt(X3))))
		self.q3[2] = 0.5*np.sqrt((1+self.acc_vals[2])*(1+(np.abs(X4)/np.sqrt(X3))))
		self.q3[3] = 0.5*np.sqrt((1+self.acc_vals[2])*(1-(np.abs(X4)/np.sqrt(X3))))
		self.q3[4] = -0.5*np.sqrt((1+self.acc_vals[2])*(1+(np.abs(X4)/np.sqrt(X3))))
		self.q3[5] = -0.5*np.sqrt((1+self.acc_vals[2])*(1-(np.abs(X4)/np.sqrt(X3))))
		self.q3[6] = 0.5*np.sqrt((1+self.acc_vals[2])*(1+(np.abs(X4)/np.sqrt(X3))))
		self.q3[7] = 0.5*np.sqrt((1+self.acc_vals[2])*(1-(np.abs(X4)/np.sqrt(X3))))

		self.q0[0] = np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[0])/np.sqrt(2)
		self.q0[1] = np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[1])/np.sqrt(2)
		self.q0[2] = np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[2])/np.sqrt(2)
		self.q0[3] = np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[3])/np.sqrt(2)
		self.q0[4] = -np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[4])/np.sqrt(2)
		self.q0[5] = -np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[5])/np.sqrt(2)
		self.q0[6] = -np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[6])/np.sqrt(2)
		self.q0[7] = -np.sqrt(self.acc_vals[2] + 1 - 2*self.q3[7])/np.sqrt(2)

		self.q1[0] = (self.acc_vals[0]*self.q3[0] + self.acc_vals[1]*self.q0[0])/(self.acc_vals[2] + 1)
		self.q1[1] = (self.acc_vals[0]*self.q3[1] + self.acc_vals[1]*self.q0[1])/(self.acc_vals[2] + 1)
		self.q1[2] = (self.acc_vals[0]*self.q3[2] + self.acc_vals[1]*self.q0[2])/(self.acc_vals[2] + 1)
		self.q1[3] = (self.acc_vals[0]*self.q3[3] + self.acc_vals[1]*self.q0[3])/(self.acc_vals[2] + 1)
		self.q1[4] = (self.acc_vals[0]*self.q3[4] + self.acc_vals[1]*self.q0[4])/(self.acc_vals[2] + 1)
		self.q1[5] = (self.acc_vals[0]*self.q3[5] + self.acc_vals[1]*self.q0[5])/(self.acc_vals[2] + 1)
		self.q1[6] = (self.acc_vals[0]*self.q3[6] + self.acc_vals[1]*self.q0[6])/(self.acc_vals[2] + 1)
		self.q1[7] = (self.acc_vals[0]*self.q3[7] + self.acc_vals[1]*self.q0[7])/(self.acc_vals[2] + 1)

		self.q2[0] = (self.acc_vals[1]*self.q3[0] - self.acc_vals[0]*self.q0[0])/(self.acc_vals[2] + 1)
		self.q2[1] = (self.acc_vals[1]*self.q3[1] - self.acc_vals[0]*self.q0[1])/(self.acc_vals[2] + 1)
		self.q2[2] = (self.acc_vals[1]*self.q3[2] - self.acc_vals[0]*self.q0[2])/(self.acc_vals[2] + 1)
		self.q2[3] = (self.acc_vals[1]*self.q3[3] - self.acc_vals[0]*self.q0[3])/(self.acc_vals[2] + 1)
		self.q2[4] = (self.acc_vals[1]*self.q3[4] - self.acc_vals[0]*self.q0[4])/(self.acc_vals[2] + 1)
		self.q2[5] = (self.acc_vals[1]*self.q3[5] - self.acc_vals[0]*self.q0[5])/(self.acc_vals[2] + 1)
		self.q2[6] = (self.acc_vals[1]*self.q3[6] - self.acc_vals[0]*self.q0[6])/(self.acc_vals[2] + 1)
		self.q2[7] = (self.acc_vals[1]*self.q3[7] - self.acc_vals[0]*self.q0[7])/(self.acc_vals[2] + 1)


		#Code quaternion selector
		J = np.inf

		for i in range(8):
			q = np.array([self.q0[i], self.q1[i], self.q2[i], self.q3[i]])
			mag = np.linalg.norm(q)
			if mag < 0.998 or mag > 1.002:
				print('Mag out of bounds')
				continue
			else: 
				mN_tilde = (1-2*self.q2[i]**2-2*self.q3[i]**2)*self.mag_vals[0] + \
						2*(self.q1[i]*self.q2[i] - self.q0[i]*self.q3[i])*self.mag_vals[1] + \
						2*(self.q1[i]*self.q3[i] + self.q0[i]*self.q2[i])*self.mag_vals[2]
				mD_tilde = (1-2*self.q1[i]**2-2*self.q2[i]**2)*self.mag_vals[2] + \
						2*(self.q0[i]*self.q1[i] + self.q2[i]*self.q3[i])*self.mag_vals[1] + \
						2*(self.q1[i]*self.q3[i] - self.q0[i]*self.q2[i])*self.mag_vals[0]
				error = 0.5*((mN - mN_tilde)**2 + (mD - mD_tilde)**2)
				if error < J:
					J = error
					self.q_min[0] = self.q0[i]
					self.q_min[1] = self.q1[i]
					self.q_min[2] = self.q2[i]
					self.q_min[3] = self.q3[i]

		rpy = self.quaternion_to_euler(self.q_min[0], self.q_min[1], self.q_min[2], self.q_min[3])
		rpy_data = [float(data) for data in rpy]
		self.mag_rpy_pub.publish(rpy_data)
	

	def quaternion_to_euler(self, x, y, z, w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.degrees(math.atan2(t0, t1))

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.degrees(math.atan2(t3, t4))
		rpy = np.array([Z, Y, X])
		return rpy


if __name__ == '__main__':
    rospy.init_node("mag_fusion_node",disable_signals=True)
    exe = Mag_Fusion()
    rospy.spin()
    