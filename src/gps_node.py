import serial 
import pynmea2
import rospy
from std_msgs.msg import String
from capstone2020.msg import gps_data
from math import sin, cos, sqrt, pi

class GPS:
    def __init__(self):
        self.ser = serial.Serial("/dev/serial0",9600,timeout=0.5) 
        self.gps_pub = rospy.Publisher("/gps_data", gps_data ,queue_size=1)

        self.gpsData = gps_data()

    def process(self):
        self.data = self.ser.readline()             
         
        if  self.data.find('GGA') > 0:   
            msg = pynmea2.parse(self.data)

            try:
                self.gpsData.latitude = self.DDM2DD(msg.lat)
                self.gpsData.altitude = self.DDM2DD(msg.lon)
                self.gpsData.altitude = msg.altitude

                self.gps_pub.publish(self.gpsData)

            except:
                pass

    def DDM2DD(self,x):
        return float(x) % 100 / 60

if __name__ == "__main__":
    rospy.init_node("GPS_reading", anonymous = True)

    try :
        gps = GPS()

        rospy.sleep(1)

        while not rospy.is_shutdown():
            gps.process()

        gps.ser.close()

    except rospy.ROSInterruptException:
        pass