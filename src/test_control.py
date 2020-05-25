#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from capstone2020.msg import Gps_data, Ppm
from capstone2020.srv import setArea
from math import sin, cos, pi, sqrt

class control:
    def __init__(self):
        # Set safety area variables
        self.areaSet = False
        self.R = None
        self.areaCenterLat, self.areaCenterLat_rad = None, None
        self.areaCenterLon, self.areaCenterLon_rad = None, None

        self.areaWarning = False  # if the location of drone is out of range, True
        self.areaRadius = None
        self.areaWidth = None
        self.areaHeight = None
        self.areaRangeGap = 5

        # Set gps variables
        self.gps_status = False
        self.curLat, self.curLat_rad = None, None
        self.curLon, self.curLon_rad = None, None
        self.curAlt = None

        # Set pose variables
        self.pose_status = False
        self.hovering = False

        self.target_lat_rad = None
        self.target_lon_rad = None
        self.target_alt = None

        self.q_w = None
        self.q_x = None
        self.q_y = None
        self.q_z = None

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
        rospy.Subscriber("/pose_covariance",PoseWithCovarianceStamped, self.kalman_cb)
        rospy.Subscriber('/input_ppm', Ppm, self.ppm_cb)

        rospy.Service('set_area', setArea, self.area_cb)
 
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

        self.q_w = msg.pose.pose.orientation.w
        self.q_x = msg.pose.pose.orientation.x
        self.q_y = msg.pose.pose.orientation.y
        self.q_z = msg.pose.pose.orientation.z

    def ppm_cb(self, msg):
        for i in range(8):
            self.input_ch.channel[i] = msg.channel[i]

    def area_cb(self, req):
        # Service callback
        self.areaSet = True

        self.shape = req.shape

        self.areaCenterLat = req.latitude
        self.areaCenterLon = req.longtitude
        self.areaOutWidth = req.width
        self.areaOutHeight = req.height
        self.areaOutRadius = req.radius

        self.areaCenterLat_rad = req.latitude * pi/180
        self.areaCenterLon_rad = req.longtitude * pi/180

        # Set Earth Radius
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter
        lat = self.areaCenterLat_rad

        self.R = sqrt(((R_long**2 * cos(lat * pi/180))**2 + (R_short**2 * sin(lat * pi/180))**2)/
               ((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))
        
        return self.areaSet

    def ppm_pub(self):
        self.output_ch.header.stamp = rospy.Time.now()
        self.output_ppm_pub.publish(self.output_ch)

    def xy_distance(self):
        self.dist_x = self.R * (self.target_lat_rad - self.curLat_rad)
        self.dist_y = self.R * cos(self.areaCenterLat_rad) * (self.target_lon_rad - self.curLon_rad)

    def inout_check(self, shape, width, height, radius):
        # in = True, out = False
        if self.shape == 1:
            if (abs(self.dist_x) < width/2) and (abs(self.dist_y) < height/2):
                return True
            else:
                return False
        else:
            if sqrt(self.dist_x**2 + self.dist_y**2) < radius:
                return True
            else:
                return False

    def process(self):
        if (self.hovering is False) and (self.input_ch.channel[5] > 1300):
            self.target_lat_rad = self.curLat_rad
            self.target_lon_rad = self.curLon_rad
            self.target_alt = self.curAlt

            self.hovering = True

        # safety area be setted up, gps signal Ok, IMU siganl Ok
        if (self.areaSet & self.gps_status & self.pose_status):
            self.xy_distance()

            if self.areaWarning is False:
                if self.inout_check(self.shape, self.areaWidth, self.areaHeight, self.areaRadius) is True:
                    self.areaWarning = False
                else:
                    self.areaWarning = True
                    self.target_lat_rad = self.areaCenterLat_rad
                    self.target_lon_rad = self.areaCenterLon_rad
                    self.target_alt = self.curAlt

                    self.PID_control()

            else:
                areaRadius = self.areaRadius - self.areaRangeGap
                areaWidth = self.areaWidth - self.areaRangeGap
                areaHeight = self.areaHeight - self.areaRangeGap

                if self.inout_check(self.shape, areaWidth, areaHeight, areaRadius) is True:
                    if self.hovering is False and self.done is False:
                        self.target_lat_rad = self.curLat_rad
                        self.target_lon_rad = self.curLon_rad

                        self.hovering = True

                    else:
                        self.areaWarning = False
                        self.hovering = False
                        self.done = False
                else:
                    self.areaWarning = True

                    self.PID_control()

        else:
            self.areaWarning = False

        if self.hovering is True:
            self.PID_control()

        if self.areaWarning is False:
            self.output_ch = self.input_ch

        self.ppm_pub()
        
    def PID_control(self):
        self.dist_z = self.target_alt - self.curAlt

        body_dist = quat_mult([self.q_w, self.q_x, self.q_y, self.q_z], quat_mult([0, self.dist_x, self.dist_y, self.dist_z], inv_quat([self.q_w, self.q_x, self.q_y, self.q_z])))
        body_dist_x = body_dist[0]
        body_dist_y = body_dist[1]
        body_dist_z = body_dist[2]
        

        self.ppm_pub()

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
    rospy.init_node('main_control')

    drone = control()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        drone.process()
        
        rate.sleep()