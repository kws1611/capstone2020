#!/usr/bin/env python

import serial 
import time
import rospy
from std_msgs.msg import String
from capstone2020.msg import gps_data

class gps:
    def __init__(self):
        self.gpss = bytearray()
        self.prevT = 0
        self.curT = 0
        self.ser = serial.serialwin32.Serial(port = "COM5")  
        self.gps_pub = rospy.Publisher("/gps_data", gps_data ,queue_size=1) 

        self.longtitude, self.latitude = 0 , 0
        ######################
        #target range set
        ##################
        
        self.target_latitude_min = 0
        self.target_latitude_max = 0
        self.target_longtitude_min = 0
        self.target_longtitude_max = 0

    def gps_reading(self):
        self.data = self.ser.read()                      
        self.gpss = self.gpss + self.data    
                         
        self.curT = time.time_ns()
        self.tdiff = self.curT-self.prevT                     
        self.prevT = self.curT

        if tdiff > 100000000:                  
            self.str_gps = self.gpss.decode('utf-8')
            self.str_gps2 = self.str_gps.split("\n")
            self.gps_data_msg = gps_data()
            try:
                self.str_gpgga = self.str_gps2[2].split(",")
                self.latitude = self.str_gpgga[2]
                self.longtitude = self.str_gpgga[4]
		        '''
                print(tdiff)
                print(str_gpgga)
                print("latitude = ",latitude,", longitue = ", longtude)
            except:
                pass
            gpss.clear()
		        '''
                self.altitude = self.str_gpgga[9]
                self.gps_time = self.str_gpgga[1]
                self.gps_data_msg.latitude = self.latitude
                self.gps_data_msg.altitude = self.altitude
                self.gps_data_msg.longtitude = self.longtitude
                self.gps_data.time = self.gps_time
                self.gps_data.range = self.range_check()
                self.gps_pub.publish(self.gps_data_msg)

    def range_check(self):
        if (self.longtitude < self.target_latitude_max) and (self.longtitude > self.target_latitude_min):
            if (self.latitude < self.target_latitude_max) and (self.latitude > self.target_latitude_min):
                return str("in")
            else:
                return str("out")
        else :
            return str("out")

if __name__ == "__main__":
    rospy.init_node("GPS_reading", anonymous = True)
    rospy.loginfo("GPS_STARTING")
    try :
        gps_function = gps()
        time.sleep(1)
        while not rospy.is_shutdown():
            gps_function.gps_reading()
        except rospy.ROSInterruptException:
            print "ROS Terminated"
            pass
    

