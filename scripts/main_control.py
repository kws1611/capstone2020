#!/usr/bin/env python

import rospy
from capstone2020.msg import Gps_data, Pose, Ppm
from capstone2020.srv import setArea
import math

class control:
    def __init__(self):
        # Set safety area variables
        self.areaSet = False
        self.areaCenterLon = None
        self.areaCenterLat = None

        self.areaRangeGap = 5
        self.areaWarning = False  # if the location of drone is out of range, True
        self.areaOutRange = None
        self.areaInRange = self.areaOutRange - self.areaRangeGap

        # Set gps variables
        self.gps_status = False
        self.curLon = None
        self.curLat = None 
        self.curAlt = None

        # Set pose variables
        self.pose_status = False
        self.curRoll = None
        self.curPitch = None
        self.curYaw = None

        # Set ppm variables
        self.input_ch = Ppm()
        self.output_ch = Ppm()
        
        # Set PID control variables
        self.P_gain = 0.5
        self.I_gain = 0.5
        self.D_gain = 0.5

        # Declare publisher, subscriber and service server
        self.output_ppm_pub = rospy.Publisher('/output_ppm', Ppm, queue_size= 1)

        rospy.Subscriber('/gps_data', Gps_data, self.gps_cb)
        rospy.Subscriber('/pose', Pose, self.pose_cb)
        rospy.Subscriber('/input_ppm', Ppm, self.ppm_cb)

        rospy.Service('set_area', setArea, self.area_cb)
 
    # subscriber's callback function
    def gps_cb(self, msg):
        self.gps_status = True

        self.curLon = msg.longtitude
        self.curLat = msg.latitude 
        self.curAlt = msg.altitude

    def pose_cb(self, msg):
        self.pose_status = True

        self.curRoll = msg.roll
        self.curPitch = msg.pitch 
        self.curYaw = msg.yaw

    def ppm_cb(self, msg):
        for i in range(8):
            self.input_ch.channel[i] = msg.channel[i]

    def area_cb(self, req):
        self.areaSet = True

        self.areaCenterLon = req.longtitude
        self.areaCenterLat = req.latitude
        self.areaOutRange = req.range
        
        return self.areaSet

    def ppm_pub(self):
        self.output_ch.header.stamp = rospy.Time.now()
        self.output_ppm_pub.publish(self.output_ch)

    # check the controller Roll = pitch = 1500, throtle > 1500
    def controller_check(self):
        roll_neutrality = abs(self.input_ch.channel[0] -1500) < 50
        pitch_neutrality = abs(self.input_ch.channel[1] -1500) < 50
        throtle_neutrality = abs(self.input_ch.channel[2] -1500) < 100

        if (roll_neutrality & pitch_neutrality & throtle_neutrality):
            return True
        else:
            return False

    # Earth long radius = 6370km, short radius = 6262km
    # Local map assume constant radius
    # Return True -> output_ppm equal to input_ppm
    # Return False -> output_ppm not equal to input_ppm
    def range_check(self):
        # safety area be setted up, gps signal Ok, IMU siganl Ok
        if (self.areaSet & self.gps_status & self.pose_status):
            diffLat = (self.areaCenterLat - self.curLat) * math.pi/180  # unit(radian)
            diffLon = (self.areaCenterLon - self.curLon) * math.pi/180  # unit(radian)

            radius = 6370-108/90*abs(self.areaCenterLat) * 10e3  # unit(m)

            areaCenterLon_rad = self.areaCenterLon * math.pi/180
            
            d = math.sqrt(pow(diffLat * radius, 2) + pow(diffLon * radius * math.cos(areaCenterLon_rad), 2))

            if self.areaWarning:
                if (d < self.areaInRange) & self.controller_check():
                    return True
                else:
                    return False

            else:
                if d < self.areaOutRange:
                    return True
                else:
                    self.areaWarning = True
                    return False

        else:
            return True

    def PID_control(self):
        self.P_gain
        self.I_gain
        self.D_gain

        '''
        dynamics
        '''

        return [0, 0, 0, 0, 0, 0, 0, 0]

if __name__ == "__main__":
    rospy.init_node('main_control')

    drone = control()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if drone.range_check():
            drone.output_ch = drone.input_ch

        else:
            drone.output_ch = drone.PID_control()

        drone.ppm_pub()
        
        rate.sleep()