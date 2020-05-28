#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from capstone2020.msg import GpsData, Ppm
from capstone2020.srv import SetArea
from math import sin, cos, pi, sqrt

from mavros_msgs.msg import RCIn
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu

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

        self.pre_inout = True  
        self.pre_hoveringSW = False
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
        self.dt = 0.05

        # Set ppm variables
        self.input_RC = Ppm()
        self.output_RC = Ppm()
        
        # Set PID control variables
        self.P_gain = 5
        self.I_gain = 0.1
        self.D_gain = 0.5
        self.error_I = 0

        # Declare publisher, subscriber and service server
        self.output_ppm_pub = rospy.Publisher('/output_ppm', Ppm, queue_size= 1)

        rospy.Subscriber('/gps_data', GpsData, self.gps_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/input_ppm', Ppm, self.ppm_cb)
        
        rospy.Service('/set_area', SetArea, self.area_cb)

    def imu_cb(self, msg):
        self.pose_status = True
        
        self.q = [msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z]
        
    # subscriber's callback function
    def gps_cb(self, msg):
        self.gps_status = True

        self.curLat = msg.latitude
        self.curLon = msg.longitude
        self.curAlt = msg.altitude

        self.curLat_rad = msg.latitude * pi/180
        self.curLon_rad = msg.longitude * pi/180

    def ppm_cb(self, msg):
        self.input_RC = msg

    def area_cb(self, req):
        # Service callback
        self.areaSet = True

        self.shape = req.shape

        self.areaCenterLat = req.latitude; self.areaCenterLat_rad = req.latitude * pi/180
        self.areaCenterLon = req.longitude; self.areaCenterLon_rad = req.longitude * pi/180

        self.areaWidth = req.width  # Unit(m)
        self.areaHeight = req.height  # Unit(m)
        self.areaRadius = req.radius  # Unit(m)

        # Set Earth Radius
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter
        lat = self.areaCenterLat_rad

        self.earth_radius = sqrt(((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

        # Set inner rectangle range in rad
        self.areaDeltaLat_rad = (self.areaHeight - 2*self.areaRangeGap) / self.earth_radius 
        self.areaDeltaLon_rad = (self.areaWidth - 2*self.areaRangeGap) / (self.earth_radius * cos(self.areaCenterLat_rad))

        return self.areaSet

    '''
    def ppm_pub(self, output_RC):
        output_RC.header.stamp = rospy.Time.now()
        self.output_ppm_pub.publish(output_RC)
    '''
    
    def hoveringSW_check(self):
        if 200 < self.input_RC.channel_7 < 700:
            self.auto_mode = True
            return True
        else:
            return False

    def inout_check(self):
        # in = True, out = False
        dist_x = self.earth_radius * cos(self.areaCenterLat_rad) * (self.areaCenterLon_rad - self.curLon_rad)
        dist_y = self.earth_radius * (self.areaCenterLat_rad - self.curLat_rad)

        if self.shape == 1:  # Rectangle
            if (abs(dist_x) < self.areaWidth/2) and (abs(dist_y) < self.areaHeight/2):
                return True
            else:
                return False

        else:  # Circle
            if sqrt(dist_x**2 + dist_y**2) < self.areaRadius:
                return True
            else:
                return False

    def set_target(self, inout, hoveringSW):
        # Hovering SW 
        if (self.pre_hoveringSW == False) and (hoveringSW == True):
            self.targetLat_rad = self.curLat_rad
            self.targetLon_rad = self.curLon_rad
            self.targetAlt = self.curAlt
            self.targetThrottle = self.input_RC.channel_3

            return

        # When get out of safety area
        if (self.pre_inout == True) and (inout == False) and (hoveringSW == False):
            minLat = self.areaCenterLat_rad - self.areaDeltaLat_rad/2
            maxLat = self.areaCenterLat_rad + self.areaDeltaLat_rad/2
            minLon = self.areaCenterLon_rad - self.areaDeltaLon_rad/2
            maxLon = self.areaCenterLon_rad + self.areaDeltaLon_rad/2

            if self.shape == 1:  # Rectangle
                self.targetLat_rad = max(minLat, min(maxLat, self.curLat_rad))
                self.targetLon_rad = max(minLon, min(maxLon, self.curLon_rad))
                self.targetAlt = self.curAlt
                self.targetThrottle = self.input_RC.channel_3

            else:  # Circle
                self.targetLat_rad = self.curLat_rad + (self.areaCenterLat_rad - self.curLat_rad) * self.areaRangeGap/self.areaRadius
                self.targetLon_rad = self.curLon_rad + (self.areaCenterLon_rad - self.curLon_rad) * self.areaRangeGap/self.areaRadius
                self.targetAlt = self.curAlt
                self.targetThrottle = self.input_RC.channel_3
        
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
        throtle_neutrality = self.input_RC.channel_3 > self.output_RC.channel_3

        return  (roll_neutrality and pitch_neutrality and throtle_neutrality)
        
    def auto_control(self, targetLat_rad, targetLon_rad, targetAlt, q, hoveringSW):
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter
        lat = self.curLat_rad

        self.earth_radius = sqrt(((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))
        ##################### dt has to be added##################################3
        dist_x = self.earth_radius * cos(self.curLat_rad) * (self.targetLon_rad - self.curLon_rad)
        dist_y = self.earth_radius * (self.targetLat_rad - self.curLat_rad)
        dist_z = self.targetAlt - self.curAlt

        d = sqrt(dist_x**2 + dist_y**2 + dist_z**2)

        rospy.loginfo_throttle(1, "x: %d"%dist_x)
        rospy.loginfo_throttle(1, "y: %d"%dist_y)
        rospy.loginfo_throttle(1, "z: %d"%dist_z)

        body_dist = quat_mult(inv_quat([q[0], 0, 0, q[3]]), quat_mult([0, dist_x, dist_y, dist_z], [q[0], 0, 0, q[3]]))

        body_dist_x = body_dist[1]
        body_dist_y = body_dist[2]
        body_dist_z = body_dist[3]

        rospy.loginfo_throttle(1, "bx: %d"%body_dist[1])
        rospy.loginfo_throttle(1, "by: %d"%body_dist[2])
        rospy.loginfo_throttle(1, "bz: %d"%body_dist[3])
        
        if(body_dist_x**2 + body_dist_y**2 > 1):
            norm_body_x = body_dist_x / sqrt(body_dist_x**2 + body_dist_y**2)
            norm_body_y = body_dist_y / sqrt(body_dist_x**2 + body_dist_y**2)
        else:
            norm_body_x =0
            norm_body_y =0
        
        ### PID loop for roll and pitch value
        # I loop
        self.error_I += self.I_gain*d*self.dt

        # making the I term not go over -1~1
        self.error_I = max(-1, min(self.error_I, 1))

        error_value = self.P_gain*d + self.error_I
        # calculating the tilt value to tilt roll and pitch
        # error value is divided by error maximum to make slower 
        # error maximum value = 5m   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        tilt_value = error_value/5*500
        
        # tilt value is between -500~500
        tilt_value = max(-500,min(tilt_value,500))
        x_tilt_value = 1000 -(norm_body_y*tilt_value)
        y_tilt_value = +(norm_body_x*tilt_value)+1000

        '''
        rospy.loginfo_throttle(1, "x: %d"%norm_body_x)
        rospy.loginfo_throttle(1, "y: %d"%norm_body_y)
        rospy.loginfo_throttle(1, "tilt: %d"%tilt_value)
        '''

        ### PID loop for altitude value
        ######## error value used for throttle
        # error maximum value check!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        throttle_value = self.targetThrottle + error_value/5*100

        # throttle value is between 350~1800
        throttle_value = max(550, min(throttle_value, 1400))

        # Reach check
        if(self.reach_check(d) and self.controller_check() and ~hoveringSW):
            self.auto_mode = False

        output = Ppm()
        output.channel_1 = x_tilt_value
        output.channel_2 = y_tilt_value
        output.channel_3 = throttle_value
        output.channel_4 = 500
        output.channel_5 = self.input_RC.channel_5
        output.channel_6 = self.input_RC.channel_6
        output.channel_7 = self.input_RC.channel_7
        output.channel_8 = self.input_RC.channel_8
        return output

    def process(self):
        if (self.gps_status is True): #and (self.pose_status is True):
            # Check channel 7 Switch
            hoveringSW = self.hoveringSW_check()
            if hoveringSW is True:
                self.auto_mode = True

            # Safety area setting check
            # If the location of drone is in of range, inout = True
            if self.areaSet is True:
                rospy.loginfo_once("area set: %d"%self.areaSet)
                inout = self.inout_check()
                if inout is False:
                    self.auto_mode = True
            else:
                inout = True
            rospy.loginfo_throttle(1, "inout: %d"%inout)
            
        else:
            self.auto_mode = False

        # If safety area were not setted output_RC = input_RC
        # If out range or auto_mode ouput_RC = auto_control
        if self.auto_mode is False:
            self.output_RC = self.input_RC
        else:
            self.set_target(inout, hoveringSW)

            rospy.loginfo_once("out Lat: %.6f"%self.curLat)
            rospy.loginfo_once("out Lon: %.6f"%self.curLon)
            rospy.loginfo_once("out Alt: %.1f"%self.curAlt)

            rospy.loginfo_once("target Lat: %.6f"%(self.targetLat_rad * 180/pi))
            rospy.loginfo_once("target Lon: %.6f"%(self.targetLon_rad * 180/pi))
            rospy.loginfo_once("target Alt: %.1f"%self.targetAlt)

            self.output_RC = self.auto_control(self.targetLat_rad, self.targetLon_rad, self.targetAlt, self.q, hoveringSW)

            self.pre_inout = inout
            self.pre_hoveringSW = hoveringSW

        rospy.loginfo_throttle(1, "auto_mode: %d"%self.auto_mode)

        self.output_RC.header.stamp = rospy.Time.now()
        self.output_ppm_pub.publish(self.output_RC)

def quat_mult(q1, q2):
        q = [0]*4

        q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
        q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
        q[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3] 
        q[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1] 

        return q

def inv_quat(q):
    return [q[0], -q[1], -q[2], -q[3]]

if __name__ == "__main__":
    rospy.init_node('test_control_rpi_node')

    drone = control()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        drone.process()
        
        rate.sleep()