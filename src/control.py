#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, PoseWithCovarianceStamped
from capstone2020.msg import ppm_msg, GpsData
from capstone2020.srv import SetArea
import numpy as np
from numpy import matrix
import time
from math import atan2, asin, sqrt, cos, sin, tan, acos, pi
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
        norm = math.sqrt(v1**2 + v2**2 + v3**2)
        out_1 = v1/sqrt(v1**2 + v2**2)
        out_2 = v2/sqrt(v1**2 + v2**2)
        out_3=  v3/norm
        return out_1, out_2, out_3

def calculating_rpy(q0,q1,q2,q3):
    # calculating quaternion -> roll pitch yaw
    # euler angle is based on x-y-z order
    quaternion_norm = sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    q0 = q0 / quaternion_norm
    q1 = q1 / quaternion_norm
    q2 = q2 / quaternion_norm
    q3 = q3 / quaternion_norm
    roll = atan2((-2*q3*q2 + 2*q1*q0),(1-2*q1**2 -2*q2**2))
    pitch = asin(2*q1*q3 + 2*q2*q0)
    yaw = atan2((-2*q1*q2 + 2*q3*q0),(1-2*q2**2 - 2*q3**2))
    return roll, pitch, yaw

class control:

    def ppm_cb(self, msg):
        self.ch1 = int(msg.channel_1)
        self.ch2 = int(msg.channel_2)
        self.ch3 = int(msg.channel_3)
        self.ch4 = int(msg.channel_4)
        self.ch5 = int(msg.channel_5)
        self.ch6 = int(msg.channel_6)
        self.ch7 = int(msg.channel_7)
        self.ch8 = int(msg.channel_8)

    def gps_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

        self.in_out = self.range_check()

    def range_check(self):
        if self.shape == 1:
            if self.target_longitude_min < self.longitude < self.target_longitude_max:
                if self.target_latitude_min < self.latitude < self.target_latitude_max:
                    return "in"
                else:
                    return "out"
            else :
                return "out"

        elif self.shape == 2:
            dx = (self.target_coordinate_lat - self.longitude)*6371000
            dy = (self.target_coordinate_long - self.latitude)*6371000
            radius = sqrt(dx^2 + dy^2)

            if radius < self.target_radius:
                return "in"
            else:
                return "out"

    def kalman_cb(self, msg):
        self.quat_x = msg.pose.pose.orientation.x
        self.quat_y = msg.pose.pose.orientation.y
        self.quat_z = msg.pose.pose.orientation.z
        self.quat_w = msg.pose.pose.orientation.w

    def area_cb(self, req):
        self.areaSet = True

        self.shape = req.shape

        self.target_coordinate_lat = req.latitude
        self.target_coordinate_long = req.longitude
        self.target_radius = req.radius

        delta_lat, delta_lon = self.meter2deg(self.latitude, req.width, req.height)

        self.target_latitude_min =  self.target_coordinate_lat - delta_lat/2
        self.target_latitude_max =  self.target_coordinate_lat + delta_lat/2

        self.target_longitude_min = self.target_coordinate_long - delta_lon/2
        self.target_longitude_max = self.target_coordinate_long + delta_lon/2

        return self.areaSet

    def meter2deg(self, lat, width, height):
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter

        R = sqrt(((R_long**2 * cos(lat * pi/180))**2 + (R_short**2 * sin(lat * pi/180))**2)/
               ((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

        delta_lat = height/R*180/pi         # x , earth radius , radian to degree , degree to minute
        delta_lon = width/(R * cos(lat*pi/180))*180/pi          # d = degree, m = minute , latitude and longitude = ddmm.mmmm format

        return delta_lat, delta_lon
    
    def deg2meter(self, lat, del_lat, del_lon):
        # changing degree difference to meter
        # lontitude -> X axis
        # latitude -> Y axis

        ########################################################## lat = home position???

        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter

        R = sqrt(((R_long**2 * cos(lat * pi/180))**2 + (R_short**2 * sin(lat * pi/180))**2)/
               ((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

        meter_X = del_lat*R*pi/180
        meter_Y = del_lon*(R*cos(lat*pi/180))*pi/180

        return meter_X, meter_Y
    
    def rotation_matrix(self,roll, pitch, yaw):
        # rotation matrix changing from global to drone frame
        # X Y Z rotation matrix
        return matrix([[1,0,0],[0, cos(roll), -sin(roll)],[0, sin(roll), cos(roll)]])*matrix([[cos(pitch), 0 , sin(pitch)],[0, 1, 0],[-sin(pitch), 0, cos(pitch)]])*matrix([[cos(yaw), -sin(yaw), 0],[sin(yaw), cos(yaw), 0],[0, 0, 1]])

    def quat_to_matrix(self,w,x,y,z):
        # changing quaternion to rotation matrix
        return matrix([[(1-2*y**2-2*z**2), (2*x*y + 2*w*z), (2*x*z - 2*w*y)],[(2*x*y - 2*w*z), (1-2*x**2-2*z**2),(2*y*z + 2*w*x)],[(2*x*z + 2*w*y),(2*y*z - 2*w*x),(1-2*x**2 -2*y**2)]])

    def __init__(self):
        # Safety area
        self.areaSet = False

        self.shape = None

        self.target_coordinate_lat = None
        self.target_coordinate_long = None
        self.radius = None

        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.in_out = "in"
        self.in_out_switch = True   # True = in Faluse = out
        self.hovering_switch = False  #True = hovering , False = radio control
        self.hov_first_switch = True #True -> False = is the time when changing the ch6 value to hovering 
        
        self.kp = 5
        self.ki = 0.01
        self.kd = 0.01
        self.dt = 1.0/50

        self.error_maximum = 5
        self.error_I = 0
        self.backup_ch3 = 400

        self.ch1,self.ch2,self.ch3,self.ch4,self.ch5,self.ch6,self.ch7,self.ch8 = 1000,1000,1000,1000,1000,1000,1000,1000
        self.quat_w,self.quat_x,self.quat_y,self.quat_z = 1,0,0,0
        self.control_ch1, self.control_ch2, self.control_ch3, self.control_ch4 = 0.0, 0.0, 0.0, 0.0
        self.I_time = time.time()
        self.des_global_x, self.des_global_y, self.des_global_z = 0.0, 0.0, 0.0
        self.des_body_x, self.des_body_y, self.des_body_z = 0.0, 0.0, 0.0

        self.back_up_switch = True
        self.back_switch = True
        self.back_up_ch1, self.back_up_ch2, self.back_up_ch3, self.back_up_ch4 = 0.0, 0.0, 0.0, 0.0

        # target position put here
        self.target_latitude_min = 0
        self.target_latitude_max = 0
        self.target_longitude_min = 0
        self.target_longitude_max = 0
        self.target_altitude = 10
        self.dist_sq = sqrt((self.target_latitude_max-self.target_latitude_min)**2 + (self.target_longitude_max-self.target_longitude_min)**2)

        # Subscriber created
        rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        rospy.Subscriber("/gps_data", GpsData, self.gps_cb)
        rospy.Subscriber("/pose_covariance",PoseWithCovarianceStamped, self.kalman_cb)
        self.controling_pub = rospy.Publisher("/control_signal", ppm_msg, queue_size=1)
        rospy.Service('set_area', SetArea, self.area_cb)

        # home position below
        self.target_coordinate_lat = (self.target_latitude_min + self.target_latitude_max)/2
        self.target_coordinate_long = (self.target_longitude_max + self.target_longitude_min)/2

    def cheching_Hovering_switch(self):
        if self.ch5 > 1300 :
            self.hovering_switch = True
            if self.hov_first_switch:
                # target the point to hover
                # the point when switch off -> switch on
                self.hov_back_up_X = self.longitude
                self.hov_back_up_Y = self.latitude
                self.hov_back_up_Z = self.altitude
                self.hov_first_switch = False
        else :
            self.hovering_switch = False
            self.hov_first_switch = True

    def checking_state(self):
        # checking state from current_state -> home position
        self.d3_error, self.norm_body_x, self.norm_body_y = self.calculating_distance_error(self.target_coordinate_long, self.target_coordinate_lat,self.back_up_altitude)
        if self.back_switch:
            if self.in_out == "out":
                self.back_up_switch = True
                self.in_out_switch = False

        if self.back_up_switch:
            self.back_up_ch1 = self.ch1
            self.back_up_ch2 = self.ch2
            self.back_up_ch3 = self.ch3
            self.back_up_ch4 = self.ch4
            self.back_up_switch = False
            self.back_up_altitude = self.altitude

        if self.in_out =="in":
            self.back_switch = True
            if (not self.in_out_switch) and (self.d3_error < 0.3):
                self.in_out_switch = True

    def calculating_distance_error(self, X, Y, Z):
        # calcculating the vector from drone to targeted position (global)
        del_lat = self.latitude - X
        del_lon = self.longitude - Y
        del_meter_X, del_meter_Y = self.deg2meter(self.target_coordinate_lat, del_lat, del_lon)

        self.des_global_x = del_meter_X
        self.des_global_y = del_meter_Y
        self.des_global_z = -Z + self.altitude
        
        # changing global vector to body frame vector
        ([[self.des_body_x],[self.des_body_y],[self.des_body_z]]) = (self.quat_to_matrix(self.quat_w,self.quat_x,self.quat_y,self.quat_z))*matrix([[self.des_global_x],[self.des_global_y],[self.des_global_z]])
        
        # normalize the vector to size 1 to calculate the roll pitch yaw
        self.norm_body_x, self.norm_body_y, self.norm_body_z = normalization(self.des_body_x,self.des_body_y,self.des_body_z)

        # pid loop error calculation part
        self.d2_dist = sqrt(self.des_global_x**2 + self.des_global_y**2)
        self.d3_dist = sqrt(self.d2_dist**2 + self.des_global_z**2)

        # making the drone to come into 2m more
        if self.dist_sq < 5:
            self.d3_error = self.d3_dist
        else :
            # calculating the safe distance = 2m when distance of box's radius is over 5m
            self.d3_error = self.d3_dist - self.dist_sq + 2

        return self.d3_error, self.norm_body_x, self.norm_body_y


    def desired_accelation(self, X, Y, Z):
        # calculating the error value and vector value 
        # from current drone coordinate -> target corrdinate(home or hovering)
        self.d3_error, self.norm_body_x, self.norm_body_y = self.calculating_distance_error(X, Y, Z)

        ### PID loop for roll and pitch value
        # I loop
        self.error_I += self.ki*self.d3_error*self.dt

        # making the I term not go over -1~1
        if self.error_I > 1 :
            self.error_I = 1
        elif self.error_I < -1:
            self.error_I = -1
        else :
            pass

        self.error_value = self.kp*self.d3_error + self.error_I  
        # calculating the tilt value to tilt roll and pitch
        # error value is divided by error maximum to make slower 
        # error maximum value = 5m
        self.tilt_value = self.error_value/self.error_maximum*500
        
        # tilt value is between -500~500
        if self.tilt_value > 500:
            self.tilt_value = 500
        elif self.tilt_value < -500:
            self.tilt_value = -500
        else :
            pass
        
        self.x_tilt_value = -(self.norm_body_y*self.tilt_value)+1500
        self.y_tilt_value = +(self.norm_body_x*self.tilt_value)+1500

        ### PID loop for altitude value
        ######## error value used for throttle
        self.throttle_value = self.backup_ch3 + self.error_value/self.error_maxium*100

        # throttle value is between 350~1800
        if self.throttle_value > 1800:
            self.throttle_value = 1800
        elif self.throttle_value < 350:
            self.throttle_value = 350
        else :
            pass
        
        return self.x_tilt_value, self.y_tilt_value, self.throttle_value

    def controling_process(self):
        self.channel_msg = ppm_msg()
        self.channel_msg.header.stamp.secs = time.time()
        self.checking_state()
        self.cheching_Hovering_switch()
        if (not self.hovering_switch) and self.in_out_switch:
            self.channel_msg.channel_1 = self.ch1
            self.channel_msg.channel_2 = self.ch2
            self.channel_msg.channel_3 = self.ch3
            self.channel_msg.channel_4 = self.ch4 

        elif self.hovering_switch and self.in_out_switch:
            self.control_ch1, self.control_ch2, self.control_ch3 = self.desired_accelation(self.hov_back_up_X, self.hov_back_up_Y, self.hov_back_up_Z)
            self.channel_msg.channel_1 = self.control_ch1
            self.channel_msg.channel_2 = self.control_ch2
            self.channel_msg.channel_3 = self.control_ch3
            self.channel_msg.channel_4 = 1500

        else :
            self.control_ch1, self.control_ch2, self.control_ch3 = self.desired_accelation(self.target_coordinate_long, self.target_coordinate_lat, self.back_up_altitude)
            self.channel_msg.channel_1 = self.control_ch1
            self.channel_msg.channel_2 = self.control_ch2
            self.channel_msg.channel_3 = self.control_ch3
            self.channel_msg.channel_4 = 1500
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