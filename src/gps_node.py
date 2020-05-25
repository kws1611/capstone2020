#!/usr/bin/python

import serial 
import pynmea2
import rospy
from capstone2020.msg import GpsData
from math import sin, cos, sqrt, pi

class GPS:
    def __init__(self):
        self.ser = serial.Serial("/dev/serial0",9600,timeout=0.5) 
        self.gps_pub = rospy.Publisher("/gps_data", GpsData ,queue_size=1)

        self.gps_data = GpsData()

    def process(self):
        self.data = self.ser.readline()             
         
        if  self.data.find('GGA') > 0:   
            msg = pynmea2.parse(self.data)

            try:
                self.gps_data.latitude = self.DDM2DD(msg.lat)
                self.gps_data.longitude = self.DDM2DD(msg.lon)
                self.gps_data.altitude = msg.altitude

                self.gps_pub.publish(self.gps_data)

            except:
                pass

    def DDM2DD(self,x):
        return float(x) % 100 / 60

if __name__ == "__main__":
    rospy.init_node("gps_node", anonymous = True)

    try :
        gps = GPS()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            gps.process()

        gps.ser.close()

    except rospy.ROSInterruptException:
        pass