#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from rise_control.msg import ppm_msg
import numpy as np
from numpy import matrix
import time
import math
from math import atan2, asin, sqrt, cos, sin, tan, acos
import numpy.linalg as lin
import tf

"""
quaternion to matrix
1-2*q2**2 - 2*q3**2 , 2*q1*q2 - 2*q3*q0 , 2*q1*q3 + 2*q2*q0
2*q1*q2 + 2*q3*q0 , 1 - 2*q1**2 - 2*q3**2 , 2*q3*q2 - 2*q1*q0
2*q1*q3 - 2*q2*q0 , 2*q2*q3 + 2*q1*q0 , 1-2*q1**2 -2*q2**2
"""
def normalization(v1, v2, v3):
        #making the vector size 1
        norm = math.sqrt(v1 ** 2 + v2 ** 2 + v3 ** 2)
        out_1 = v1 / (v1**2 + v2**2)
        out_2 = v2 / (v1**2 + v2**2)
        out_3=  v3 / norm
        return out_1, out_2, out_3

def calculating_rpy(q0,q1,q2,q3):
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

class control:

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

    def gps_cb(self, msg):
        self.gps_msg = msg
        self.latitude = self.gps_msg.latitude
        self.longtitude = self.gps_msg.longtitude
        self.altitude = self.gps_msg.altitude
        self.in_out = self.gps_msg.range

    def kalman_cb(self, msg):
        self.kalman_msg = msg
        self.quat_x = self.kalman_msg.pose.pose.orientation.x
        self.quat_y = self.kalman_msg.pose.pose.orientation.y
        self.quat_z = self.kalman_msg.pose.pose.orientation.z
        self.quat_w = self.kalman_msg.pose.pose.orientation.w

    def rotation_matrix(roll, pitch, yaw):
        # rotation matrix changing from global to drone frame
        # X Y Z rotation matrix
        return matrix([[1,0,0],[0, cos(roll), -sin(roll)],[0, sin(roll), cos(roll)]])*matrix([[cos(pitch), 0 , sin(pitch)],[0, 1, 0],[-sin(pitch), 0, cos(pitch)]])*matrix([[cos(yaw), -sin(yaw), 0],[sin(yaw), cos(yaw), 0],[0, 0, 1]])

    def quat_to_matrix(w,x,y,z):
        # changing quaternion to rotation matrix
        return matrix([[(1-2*y**2-2*z**2), (2*x*y + 2*w*z), (2*x*z - 2*w*y)],[(2*x*y - 2*w*z), (1-2*x**2-2*z**2),(2*y*z + 2*w*x)],[(2*x*z + 2*w*y),(2*y*z - 2*w*x),(1-2*x**2 -2*y**2)]])

    def __init__(self, x, y ,z):
        self.latitude = 0
        self.longtitude = 0
        self.altitude = 0
        self.in_out = ""
        ### has to be added
        self. dist_sq = 0

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
        self.control_ch1, self.control_ch2, self.control_ch3, self.control_ch4 = 0.0, 0.0, 0.0, 0.0
        self.error_x,self.error_y, self.error_z = 0.0, 0.0, 0.0
        self.prev_error_x, self.prev_error_y, self.prev_error_z = 0.0, 0.0, 0.0
        self.error_roll, self.error_pitch, self.error_yaw = 0.0, 0.0, 0.0
        self.prev_roll, self.prev_pitch, self.prev_yaw = 0.0, 0.0 ,0.0
        self.x_I, self.y_I, self.z_I = 0.0, 0.0, 0.0
        self.roll_I, self.pitch_I, self.yaw_I = 0.0, 0.0 , 0.0
        self.roll, self.pitch, self.yaw, self.throttle = 0.0, 0.0, 0.0, 0.0
        self.I_time = time.time()
        self.des_global_x, self.des_global_y, self.des_global_z = 0.0, 0.0, 0.0
        self.des_body_x, self.des_body_y, self.des_body_z = 0.0, 0.0, 0.0

        self.back_up_switch = True
        self.back_switch = True
        self.back_up_ch1, self.back_up_ch2, self.back_up_ch3, self.back_up_ch4 = 0.0, 0.0, 0.0, 0.0

        # target position put here
        ###########################################################################
        self.target_latitude_min = 0
        self.target_latitude_max = 0
        self.target_longtitude_min = 0
        self.target_longtitude_max = 0
        self.target_altitude = 0
        #############################################################################

        self.target_coordinate_lat = (self.target_latitude_max + self.target_latitude_max)/2
        self.target_coordinate_long = (self.target_longtitude_max + self.target_longtitude_min)/2

        # Subscriber created
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        rospy.Subscriber("/gps_data", gps_data, self.gps_cb)
        rospy.Subscriber("/pose_covariance",PoseWithCovarianceStamped, self.kalman_cb)

        self.controling_pub = rospy.Publisher("/control_signal", ppm_msg, queue_size=1)

    def checking_state(self):
        if self.back_switch:
            if self.in_out == "out":
                self.back_up_switch = True

        if self.back_up_switch:
            self.back_up_ch1 = self.ch1
            self.back_up_ch2 = self.ch2
            self.back_up_ch3 = self.ch3
            self.back_up_ch4 = self.ch4
            self.back_up_switch = False

        if self.in_out =="in":
            self.back_switch = True
    '''
    def calculating_desired(self,x_des,y_des,z_des):
        phi_desired = atan2(-y_des, z_des)
        theta_desired = atan2(x_des, -y_des/sin(phi_desired))
        throttle = z_des/(cos(phi_desired)*cos(theta_desired))
        return phi_desired, theta_desired, throttle
    '''
    def desired_accelation(self):
        # calcculating the vector from drone to targeted position (global)
        self.des_global_x = self.target_coordinate_long - self.longtitude
        self.des_global_y = self.target_coordinate_lat - self.latitude
        self.des_global_z = self.target_altitude - self.altitude
        
        # changing global vector to body frame vector
        matrix([[self.des_body_x],[self.des_body_y],[self.des_body_z]]) = self.quat_to_matrix(self.quat_w,self.quat_x,self.quat_y,self.quat_z)*matrix([[self.des_global_x],[self.des_global_y],[self.des_global_z]])
        # normalize the vector to size 1 to calculate the roll pitch yaw
        self.norm_body_x, self.norm_body_y, self.norm_body_z = normalization(self.des_body_x,self.des_body_y,self.des_body_z)

        # pid loop
        self.2d_dist = sqrt(self.des_global_x**2 + self.global_y**2)
        self.3d_dist = sqrt(self.2d_dist**2 + self.des_global_z**2)
        self.3d_error = self.3d_error - self.dist_sq

        # I loop
        self.error_I += self.ki*self.3d_dist*self.dt

        # calculating the safe distance = 2m
        self.error_value = self.kp*self.3d_error + self.error_I 


        ## 



        self.x_tilt_value = -(self.norm_body_y)
        self.y_tilt_value = +(self.norm_body_x)
        self.throttle_value = self.backup_ch3 + self.norm_body_z














        # integrating the I term from(PID) and erase it every 3 sec
        self.x_I += self.error_x * self.dt
        self.y_I += self.error_y * self.dt
        self.z_I += self.error_z * self.dt
        if (time.time() - self.I_time) > 3 :
            self.x_I, self.y_I, self.z_I = 0.0, 0.0, 0.0
            self.I_time = time.time()

        self.prev_error_x, self.prev_error_y, self.prev_error_z = self.error_x, self.error_y, self.error_z
        self.phi_desired ,self.theta_desired, self.throttle = self.calculating_desired(self.x_desired_accel,self.y_desired_accel,self.z_desired_accel)
        return self.x_tilt_value, self.y_tilt_value, self.throttle_value

    def controling_process(self):
        self.channel_msg = ppm_msg()
        self.channel_msg.header.stamp = time.time()
        if self.in_out == "in":
            self.channel_msg.channel_1 = self.ch1
            self.channel_msg.channel_2 = self.ch2
            self.channel_msg.channel_3 = self.ch3
            self.channel_msg.channel_4 = self.ch4 

        else :
            self.calculating()
            self.channel_msg.channel_1 = self.control_ch1
            self.channel_msg.channel_2 = self.control_ch2
            self.channel_msg.channel_3 = self.control_ch3
            self.channel_msg.channel_4 = self.control_ch4
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
        drone_control = control()
        while not rospy.is_shutdown():
            drone_control.controling_process()
    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass