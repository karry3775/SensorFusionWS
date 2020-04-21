#!/usr/bin/env python
import rospy
import numpy as np
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import matplotlib.pyplot as plt
import math as m

def extractData(dir):
    # read data from file
    data = []
    with open(dir) as csvfile:
        read_csv = csv.reader(csvfile, delimiter=',')
        for row in read_csv:
            temp = [float(row[0]), float(row[1]), float(row[2])]
            data.append(temp)
    return data

def cvtToradians(data):
    data_new = []
    for vals in data:
        r, p, y = vals
        r = m.radians(r)
        p = m.radians(p)
        y = m.radians(y)
        temp = [r, p, y]
        data_new.append(temp)
    return data_new

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

# Data
gyro_data = extractData("../data/gyro.csv")
gyro_data = cvtToradians(gyro_data)
accel_data = extractData("../data/acc.csv")
mag_data = extractData("../data/mag.csv")
gt_data = extractData("../data/groundtruthOrientation.csv")
length = len(gyro_data)

# Weights
g_wt = 0.75
am_wt = 1 - g_wt

# CF filter varibles
CF_roll = []
CF_pitch = []
CF_yaw = []

# GT variables
GT_roll = []
GT_pitch = []
GT_yaw = []

# corrections
roll_correction = None
pitch_correction = None
yaw_correction =  None
q = None
dt = 1.0 / 100.0

for i in range(length):
    # extract data
    ax, ay, az = accel_data[i]
    mx, my, mz = mag_data[i]
    wx, wy, wz = gyro_data[i]

    ## normalize data
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

    if i == 0:
        CF_roll.append(rollA)
        CF_pitch.append(pitchA)
        CF_yaw.append(yawM)

        # find the corrective terms
        roll_gt, pitch_gt, yaw_gt = gt_data[i]
        # conver to radians
        roll_gt = m.radians(roll_gt)
        pitch_gt = m.radians(pitch_gt)
        yaw_gt = m.radians(yaw_gt)

        roll_correction = wrapToPi(roll_gt - rollA)
        pitch_correcton = wrapToPi(pitch_gt - pitchA)
        yaw_correction = wrapToPi(yaw_gt - yawM)
        q = quaternion_from_euler(rollA, pitchA, yawM)

    else:
        # process gyroscope data
        current_gyro_quat = quaternion_from_euler(wx * dt, wy * dt, wz * dt)
        q = quaternion_multiply(current_gyro_quat, q)
        rollG, pitchG, yawG = euler_from_quaternion(q)

        ### COMPLIMENTARY FILTER ###
        rollF = rollG * g_wt + rollA * am_wt
        pitchF = pitchG * g_wt + pitchA * am_wt
        yawF = yawG * g_wt + yawM * am_wt

        CF_roll.append(wrapToPi(rollF + roll_correction))
        CF_pitch.append(wrapToPi(pitchF + pitch_correcton))
        CF_yaw.append(wrapToPi(yawF + yaw_correction))


# process gt data
for i in range(len(gt_data)):
    roll, pitch, yaw = gt_data[i]
    roll = m.radians(roll)
    pitch = m.radians(pitch)
    yaw = m.radians(yaw)

    GT_roll.append(roll)
    GT_pitch.append(pitch)
    GT_yaw.append(yaw)

# plot
fig, axs = plt.subplots(3)
axs[0].set_title("ROLL")
axs[0].plot(CF_roll, "b-")
axs[0].plot(GT_roll, "r-")

axs[1].set_title("PITCH")
axs[1].plot(CF_pitch, "b-")
axs[1].plot(GT_pitch, "r-")

axs[2].set_title("YAW")
axs[2].plot(CF_yaw, "b-")
axs[2].plot(GT_yaw, "r-")

plt.show()
