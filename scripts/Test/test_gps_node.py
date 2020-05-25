#!/usr/bin/python

import rospy
from sensor_msgs.msg import NavSatFix
from capstone2020.msg import GpsData

class GPS:
    def __init__(self):
        self.fix = None
        self.gps_data = GpsData()

        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.positionCb)

        self.gps_pub = rospy.Publisher("/gps_data", GpsData ,queue_size=1)

    def process(self):
        if self.fix == 0:
            self.gps_data.header.stamp = rospy.Time.now()
            self.gps_pub.publish(self.gps_data)

    def positionCb(self, msg):
        self.fix = msg.status.status 
        self.gps_data.latitude = msg.latitude
        self.gps_data.longitude = msg.longitude
        self.gps_data.altitude = msg.altitude

if __name__ == "__main__":
    rospy.init_node("test_gps_node", anonymous = True)

    try :
        gps = GPS()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            gps.process()

    except rospy.ROSInterruptException:
        pass