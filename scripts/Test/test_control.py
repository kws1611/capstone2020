#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from capstone2020.msg import GpsData, Ppm
from capstone2020.srv import setArea
from math import sin, cos, pi, sqrt

class control:
    def __init__(self):
        # Set safety area variables
        self.areaSet = False

        self.areaCenterLat, self.areaCenterLat_rad = None, None
        self.areaCenterLon, self.areaCenterLon_rad = None, None

        self.areaWidth = None; self.areaHeight = None; self.areaRadius = None
        self.areaDeltaLat_rad = None; self.areaDeltaLon_rad = None
        self.areaRangeGap = 10

        self.earth_radius = None

        self.inout = True; self.pre_inout = True  # if the location of drone is in of range, True
        self.hoveringSW = False; self.pre_hoveringSW = False
        self.auto_mode = False  # Auto mode
        
        # Set gps variables
        self.gps_status = False
        self.curLat, self.curLat_rad = None, None
        self.curLon, self.curLon_rad = None, None
        self.curAlt = None

        # Set pose variables
        self.pose_status = False

        self.targetLat_rad = None
        self.targetLon_rad = None
        self.targetAlt = None

        self.q = [None]*4  # w, x, y, z

        self.time_sw = False
        self.ref_time = None

        # Set ppm variables
        self.input_RC = Ppm()
        self.output_RC = Ppm()
        
        # Set PID control variables
        self.P_gain = 0.5
        self.I_gain = 0.5
        self.D_gain = 0.5

        # Declare publisher, subscriber and service server
        self.output_ppm_pub = rospy.Publisher('/output_ppm', Ppm, queue_size= 1)

        rospy.Subscriber('/gps_data', GpsData, self.gps_cb)
        rospy.Subscriber("/pose_covariance",PoseWithCovarianceStamped, self.kalman_cb)
        rospy.Subscriber('/input_ppm', Ppm, self.ppm_cb)

        rospy.Service('/set_area', setArea, self.area_cb)
 
    # subscriber's callback function
    def gps_cb(self, msg):
        self.gps_status = True

        self.curLat = msg.latitude
        self.curLon = msg.longtitude
        self.curAlt = msg.altitude

        self.curLat_rad = msg.latitude * pi/180
        self.curLon_rad = msg.longtitude * pi/180

    def kalman_cb(self, msg):
        self.pose_status = True

        self.q = [msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z]

    def ppm_cb(self, msg):
        for i in range(8):
            self.input_RC.channel[i] = msg.channel[i]

    def area_cb(self, req):
        # Service callback
        self.areaSet = True

        self.shape = req.shape

        self.areaCenterLat = req.latitude; self.areaCenterLat_rad = req.latitude * pi/180
        self.areaCenterLon = req.longtitude; self.areaCenterLon_rad = req.longtitude * pi/180

        self.areaWidth = req.width  # Unit(m)
        self.areaHeight = req.height  # Unit(m)
        self.areaRadius = req.radius  # Unit(m)

        # Set Earth Radius
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter
        lat = self.areaCenterLat_rad

        self.earth_radius = sqrt(((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

        # Set inner rectangle range in rad
        self.areaDeltaLat_rad = (self.areaWidth - self.areaRangeGap) / (self.earth_radius * cos(self.areaCenterLat_rad))
        self.areaDeltaLon_rad = (self.areaHeight - self.areaRangeGap) / self.earth_radius

        return self.areaSet

    def ppm_pub(self, output_RC):
        output_RC.header.stamp = rospy.Time.now()
        self.output_ppm_pub.publish(output_RC)

    def hoveringSW_check(self):
        if self.input_RC.channel[6] > 1300:
            self.auto_mode = True
            return True
        else:
            return False

    def inout_check(self):
        # in = True, out = False
        dist_x = self.earth_radius * cos(self.areaCenterLat_rad) * (self.areaCenterLat_rad - self.curLat_rad)
        dist_y = self.earth_radius * (self.areaCenterLon_rad - self.curLon_rad)

        if self.shape == 1:  # Rectangle
            if (abs(dist_x) < self.areaWidth/2) and (abs(dist_y) < self.areaHeight/2):
                return True
            else:
                self.auto_mode = True
                return False

        else:  # Circle
            if sqrt(dist_x**2 + dist_y**2) < self.areaRadius:
                return True
            else:
                self.auto_mode = True
                return False

    def set_target(self):
        # Hovering SW 
        if (self.pre_hoveringSW == True) and (self.hoveringSW == True):
            self.targetLat_rad = self.curLat_rad
            self.targetLon_rad = self.curLon_rad
            self.targetAlt = self.curAlt

            return

        # When get out of safety area
        minLat = self.areaCenterLat_rad - self.areaDeltaLat_rad/2
        maxLat = self.areaCenterLat_rad - self.areaDeltaLat_rad/2
        minLon = self.areaCenterLat_rad - self.areaDeltaLat_rad/2
        maxLon = self.areaCenterLat_rad - self.areaDeltaLat_rad/2

        if (self.pre_inout == True) and (self.inout == False):
            if self.shape == 1:  # Rectangle
                self.targetLat_rad = max(minLat, min(maxLat, self.curLat_rad))
                self.targetLon_rad = max(minLon, min(maxLon, self.curLon_rad))
                self.targetAlt = self.curAlt

            else:  # Circle
                self.targetLat_rad = self.curLat_rad + (self.areaCenterLat_rad - self.curLat_rad) * self.areaRangeGap/self.areaRadius
                self.targetLon_rad = self.curLon_rad + (self.areaCenterLon_rad - self.curLon_rad) * self.areaRangeGap/self.areaRadius
                self.targetAlt = self.curAlt

    def process(self):
        self.hoveringSW = self.hoveringSW_check()
        self.inout = self.inout_check()

        if (self.inout == True) and (self.auto_mode == False):
            self.output_RC = self.input_RC
        else:
            self.set_target()
            rospy.loginfo_once("target Lat: ", self.targetLat_rad * 180/pi)
            rospy.loginfo_once("target Lon: ", self.targetLon_rad * 180/pi)
            rospy.loginfo_once("target Alt: ", self.targetAlt)

            self.output_RC = self.input_RC #self.auto_control(self.targetLat_rad, self.targetLon_rad, self.targetAlt, self.q)

        print("inout: ", self.inout)
        
        self.pre_inout = self.inout
        self.pre_hoveringSW = self.hoveringSW

        self.ppm_pub(self.output_RC)

    def auto_control(self, targetLat_rad, targetLon_rad, targetAlt, q):
        dist_x = self.earth_radius * cos(self.areaCenterLat_rad) * (self.targetLat_rad - self.curLat_rad)
        dist_y = self.earth_radius * (self.targetLon_rad - self.curLon_rad)
        dist_z = self.targetAlt - self.curAlt

        d = sqrt(dist_x**2 + dist_y**2 + dist_z**2)

        body_dist = quat_mult([q[0], q[1], q[2], q[3]], quat_mult([0, dist_x, dist_y, dist_z], inv_quat([q[0], q[1], q[2], q[3]])))
        
        body_dist_x = body_dist[0]
        body_dist_y = body_dist[1]
        body_dist_z = body_dist[2]

        # Reach check
        if(self.reach_check(d) and self.controller_check() and ~self.hoveringSW):
            self.auto_mode = False

        return self.input_RC
    
    def reach_check(self, d, _range = 1):
        if (d < _range):
            if(self.time_sw is False):
                self.ref_time = rospy.Time.now()

            if(rospy.Time.now() - self.ref_time > rospy.Duration(5.0)):
                self.time_sw = True
                return True

            else:
                return False
        else:
            self.time_sw = False
            return False

    def controller_check(self):
        roll_neutrality = abs(self.input_RC.channel[0] -1500) < 50
        pitch_neutrality = abs(self.input_RC.channel[1] -1500) < 50
        throtle_neutrality = self.input_RC.channel[2] > self.output_RC[2]

        return  (roll_neutrality and pitch_neutrality and throtle_neutrality)

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
    rospy.init_node('test_control_node')

    drone = control()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        drone.process()
        
        rate.sleep()
