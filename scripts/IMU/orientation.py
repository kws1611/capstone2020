#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time
from math import sqrt
import numpy as np
from MPU9250 import MPU

class orientation:
    def __init__(self, acc, gyro, mag):
        self.X = self.observation(acc, mag)
        self.Xp = None

        self.P = np.identity(4)
        self.Pp = None

        self.Z = None

        self.Q = 10.0**(-10)*np.matrix([[1.0, 0, 0, 0],[0, 1.50628058**2, 0, 0],[0, 0, 1.4789602**2, 0],[0, 0, 0, 1.37315181**2]])   # process noise
        self.R = 5*np.matrix([[0.00840483082215**2, 0, 0, 0],[0, 0.00100112198402**2, 0, 0], [0, 0, 0.00102210818946**2, 0], [0, 0, 0, 0.0114244938775**2]])   # observation noise
        
        self.roll = None
        self.pitch = None
        self.yaw = None

        self.dt = 0.01

        self.pose = PoseStamped()
        self.KF_pub = rospy.Publisher('/pose', PoseStamped, queue_size= 1)

    def observation(self, acc, mag):
        acc_norm = sqrt(sum([i**2 for i in acc]))
        acc = [i/acc_norm for i in acc]

        if acc[2] >= 0:
            q_acc = [sqrt((acc[2]+1) / 2), -acc[1] / sqrt(2*(acc[2]+1)), acc[0] / sqrt(2*(acc[2]+1)), 0]
        else:
            q_acc = [-acc[1] / sqrt(2*(1 - acc[2])), sqrt((1 - acc[2]) / 2), 0, acc[0] / sqrt(2*(1 - acc[2]))]

        l = quat_mult(quat_mult(inv_quat(q_acc), [0] + mag), q_acc)
        b = sqrt(l[1]**2 + l[2]**2)

        if l[1] >= 0:
            q_mag = [sqrt(0.5*(l[1]/b + 1)), 0, 0, l[2] / sqrt(2*b*(l[1] + b))]
        else:
            q_mag = [l[2] / sqrt(2*b*(b - l[1])), 0, 0, sqrt(0.5*(1 - l[1]/b))]

        tmp = quat_mult(q_acc, q_mag)

        return np.matrix([[tmp[0]], [tmp[1]], [tmp[2]], [tmp[3]]])

    def Kalman(self, acc, gyro, mag):
        F = np.identity(4) + 0.5 * self.dt * np.matrix([[0, -gyro[0], -gyro[1], -gyro[2]],
                                                        [gyro[0], 0, gyro[2], -gyro[1]],
                                                        [gyro[1], -gyro[2], 0, gyro[0]],
                                                        [gyro[2], gyro[1], -gyro[0], 0]])
        H = np.identity(4)

        self.Xp = F * self.X
        self.Pp = F * self.P * F.T + self.Q

        self.Xp /= np.linalg.norm(self.Xp)

        Z = self.observation(acc, mag)

        K = self.Pp * H.T * np.linalg.inv(H*self.Pp*H.T + self.R)

        self.X = self.Xp + K * (Z - H*self.Xp)
        self.X /= np.linalg.norm(self.X)
        self.P = (np.identity(4) - K*H) * self.Pp

    def publish(self):
        self.pose.header.stamp = rospy.Time.now()

        self.pose.header.frame_id = "base_link"

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0

        self.pose.pose.orientation.w = self.X[0,0]
        self.pose.pose.orientation.x = self.X[1,0]
        self.pose.pose.orientation.y = self.X[2,0]
        self.pose.pose.orientation.z = self.X[3,0]

        self.KF_pub.publish(self.pose)

def quat_mult(q1, q2):
        q = [0]*4

        q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
        q[1] = q1[1]*q2[0] + q1[0]*q2[1] + q1[3]*q2[2] - q1[2]*q2[3]
        q[2] = q1[2]*q2[0] - q1[3]*q2[1] + q1[0]*q2[2] + q1[1]*q2[3]
        q[3] = q1[3]*q2[0] + q1[2]*q2[1] - q1[1]*q2[2] + q1[0]*q2[3]

        return q

def inv_quat(q):
    return [q[0], -q[1], -q[2], -q[3]]

if __name__ == "__main__":
    rospy.init_node("Orientation")

    # Set up MPU class
    mpu = MPU(500, 4, 16)

    bias = rospy.get_param("/mag_bias", None)
    scale = rospy.get_param("/mag_scale", None)

    if (bias == None) or (scale == None):
        rospy.loginfo("Magnetometer Should be Calibrated")
        rospy.loginfo("Run \"mag_calibration.py\" Node")
        quit()
    else:
        mpu.setMagCalibration(bias, scale)
        mpu.calibrateGyro(1000)

    # Set up orientation class
    acc, gyro, mag = mpu.processValues()
    ori = orientation(acc, gyro, mag)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            acc, gyro, mag = mpu.processValues()
            ori.Kalman(acc, gyro, mag)
            ori.publish()

            rate.sleep()

        except:
            pass