#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from rise_control.msg import ppm_msg
import numpy as np
import time
import math
from math import atan2, asin, sqrt
import numpy.linalg as lin
import tf

"""
quaternion to matrix
1-2*q2**2 - 2*q3**2 , 2*q1*q2 - 2*q3*q0 , 2*q1*q3 + 2*q2*q0
2*q1*q2 + 2*q3*q0 , 1 - 2*q1**2 - 2*q3**2 , 2*q3*q2 - 2*q1*q0
2*q1*q3 - 2*q2*q0 , 2*q2*q3 + 2*q1*q0 , 1-2*q1**2 -2*q2**2
"""

class control:
    def calculating_rpy(self,q0,q1,q2,q3):
        # calculating quaternion -> roll pitch yaw
        # euler angle is based on x-y-z order
        quaternion_norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        q0 = q0 / quaternion_norm
        q1 = q1 / quaternion_norm
        q2 = q2 / quaternion_norm
        q3 = q3 / quaternion_norm
        roll = math.atan2((-2*q3*q2 + 2*q1*q0),(1-2*q1**2 -2*q2**2))
        pitch = math.asin(2*q1*q3 + 2*q2*q0)
        yaw = math.atan2((-2*q1*q2 + 2*q3*q0),(1-2*q2**2 - 2*q3**2))
        return roll, pitch, yaw

    def motion_cb(self, msg):
        self.mot_msg = msg
        self.motion_time = self.mot_msg.header.stamp.secs + self.mot_msg.header.stamp.nsecs * 10 ** (-9)
        self.motion_quat_x = self.mot_msg.pose.orientation.x
        self.motion_quat_y = self.mot_msg.pose.orientation.y
        self.motion_quat_z = self.mot_msg.pose.orientation.z
        self.motion_quat_w = self.mot_msg.pose.orientation.w
        self.motion_x = self.mot_msg.pose.position.x
        self.motion_y = self.mot_msg.pose.position.y
        self.motion_z = self.mot_msg.pose.position.z

    def ppm_cb(self, msg):
        self.ppm_input_msg = msg
        self.ch1 = int(self.ppm_input_msg.channel_1)
        self.ch2 = int(self.ppm_input_msg.channel_2)
        self.ch3 = int(self.ppm_input_msg.channel_3)
        self.ch4 = int(self.ppm_input_msg.channel_4)
        self.ch5 = int(self.ppm_input_msg.channel_5)
        self.ch6 = int(self.ppm_input_msg.channel_6)
        self.ch7 = int(self.ppm_input_msg.channel_7)
        self.ch8 = int(self.ppm_input_msg.channel_8)

    def __init__(self, x, y ,z):
        # reference point
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = 0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.dt = 1/50

        self.ch1 = 1000
        self.ch2 = 1000
        self.ch3 = 1000
        self.ch4 = 1000
        self.ch5 = 1000
        self.ch6 = 1000
        self.ch7 = 1000
        self.ch8 = 1000
        self.motion_x = 0.0
        self.motion_y = 0.0
        self.motion_z = 0.0
        self.motion_quat_x = 0.0
        self.motion_quat_y = 0.0
        self.motion_quat_z = 0.0
        self.motion_quat_w = 0.0
        self.motion_time = 0.0
        self.motion_time_prev = 0.0
        self.motion_roll = 0.0
        self.motion_pitch = 0.0
        self.motion_yaw = 0.0
        self.error_x,self.error_y, self.error_z = 0.0, 0.0, 0.0
        self.prev_error_x, self.prev_error_y, self.prev_error_z = 0.0, 0.0, 0.0
        self.error_roll, self.error_pitch, self.error_yaw = 0.0, 0.0, 0.0
        self.prev_roll, self.prev_pitch, self.prev_yaw = 0.0, 0.0 ,0.0
        self.x_I, self.y_I, self.z_I = 0.0, 0.0, 0.0
        self.roll_I, self.pitch_I, self.yaw_I = 0.0, 0.0 , 0.0
        self.roll, self.pitch, self.yaw, self.throttle = 0.0, 0.0, 0.0, 0.0
        # Subscriber created
        #rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose", PoseStamped, self.motion_cb)
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        self.controling_pub = rospy.Publisher("/control_signal", ppm_msg, queue_size=1)

    def calculating_desired(self,x_des,y_des,z_des):
        phi_desired = atan2(-y_des, z_des)
        theta_desired = atan2(x_des, -y_des/sin(phi_desired))
        throttle = z_des/(cos(phi_desired)*cos(theta_desired))

        return phi_desired, theta_desired, throttle

    def desired_accelation(self):
        self.error_x = self.target_x - self.motion_x
        self.error_y = self.target_y - self.motion_y
        self.error_z = self.target_z - self.motion_z
        self.x_I += self.error_x * self.dt
        self.y_I += self.error_y * self.dt
        self.z_I += self.error_z * self.dt

        self.x_desired_accel = self.kp * self.error_x + self.kd * (self.error_x - self.prev_error_x)/self.dt + self.ki *(self.x_I)
        self.y_desired_accel = self.kp * self.error_y + self.kd * (self.error_y - self.prev_error_y) / self.dt + self.ki * (self.y_I)
        self.z_desired_accel = self.kp * self.error_z + self.kd * (self.error_z - self.prev_error_z) / self.dt + self.ki * (self.z_I)

        self.prev_error_x, self.prev_error_y, self.prev_error_z = self.error_x, self.error_y, self.error_z

        self.phi_desired ,self.theta_desired, self.throttle = self.calculating_desired(self.x_desired_accel,self.y_desired_accel,self.z_desired_accel)

        self.x_tilt_value = self.phi_desired*50 + 1000
        self.y_tilt_value = self.theta_desired*50 + 1000
        self.throttle_value = self.throttle*50 + 250

        return self.x_tilt_value, self.y_tilt_value, self.throttle_value

    def controling_process(self):
        #self.pid_ch2 , self.pid_ch1, self.pid_ch3 = self.desired_accelation
        #self.pid_ch4 = 0.0
        self.channel_msg = ppm_msg()
        #self.channel_msg.header.stamp = time.time()
        if self.ch5 > 1350:
            self.channel_msg.channel_1 = self.ch1 + 50
            self.channel_msg.channel_2 = self.ch2 + 50
            self.channel_msg.channel_3 = self.ch3 + 50
            self.channel_msg.channel_4 = self.ch4 + 50
            self.channel_msg.channel_5 = self.ch5
            self.channel_msg.channel_6 = self.ch6
            self.channel_msg.channel_7 = self.ch7
            self.channel_msg.channel_8 = self.ch8
        elif self.ch5 < 700 :
            self.channel_msg.channel_1 = self.ch1 - 50
            self.channel_msg.channel_2 = self.ch2 - 50
            self.channel_msg.channel_3 = self.ch3 - 50
            self.channel_msg.channel_4 = self.ch4 - 50
            self.channel_msg.channel_5 = self.ch5
            self.channel_msg.channel_6 = self.ch6
            self.channel_msg.channel_7 = self.ch7
            self.channel_msg.channel_8 = self.ch8

        else :
            self.channel_msg.channel_1 = self.ch1
            self.channel_msg.channel_2 = self.ch2
            self.channel_msg.channel_3 = self.ch3
            self.channel_msg.channel_4 = self.ch4
            self.channel_msg.channel_5 = self.ch5
            self.channel_msg.channel_6 = self.ch6
            self.channel_msg.channel_7 = self.ch7
            self.channel_msg.channel_8 = self.ch8

        self.controling_pub.publish(self.channel_msg)

if __name__ == "__main__":
    rospy.init_node("controlling_node", anonymous=True)
    rospy.loginfo("control node initialized")
    try:
        rospy.loginfo("controling start!")
        drone_control = control(1,1,1)
        while not rospy.is_shutdown():
            drone_control.controling_process()

    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass


