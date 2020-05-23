#!/usr/bin/python

import rospy
from sensor_msgs.msg import NavSatFix
from capstone2020.msg import gps_data

class GPS:
    def __init__(self):
        self.fix = None
        self.gpsData = gps_data()

        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.positionCb)

        self.gps_pub = rospy.Publisher("/gps_data", gps_data ,queue_size=1)

    def process(self):
        if self.fix == 0:
            self.gps_pub.publish(self.gpsData)

    def positionCb(self, msg):
        self.fix = msg.status.status 
        self.gpsData.latitude = msg.latitude
        self.gpsData.longitude = msg.longitude
        self.gpsData.altitude = msg.altitude

if __name__ == "__main__":
    rospy.init_node("simulation_gps_node", anonymous = True)

    try :
        gps = GPS()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            gps.process()

    except rospy.ROSInterruptException:
        pass