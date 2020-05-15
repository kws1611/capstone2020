import serial 
import time
import rospy
from std_msgs.msg import String
from capstone2020.msg import gps_data
import numpy as np

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
        self.init_long = 0
        self.init_lat = 0
        self.safe_area_shape = ''
        self.target_latitude_min = 0
        self.target_latitude_max = 0
        self.target_longtitude_min = 0
        self.target_longtitude_max = 0
        self.target_radius = 0

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
        if shape == 'rec' or shape == 'square':
            if (self.longtitude < self.target_latitude_max) and (self.longtitude > self.target_latitude_min):
                if (self.latitude < self.target_latitude_max) and (self.latitude > self.target_latitude_min):
                    return str("in")
                else:
                    return str("out")
            else :
                return str("out")
            
        elif shape == 'circle':
            dx = (self.init_long-self.longtitude)*6371000
            dy = (self.init_lat-self.longtitude)*6371000
            radius = np.sqrt(dx^2 + dy^2)
            if radius < self.target_radius:
                return str("in")
            else:
                return str("out")


class safe_area:
    def __init__(self):
        self.shape = ''
        self.r = 0
        self.x = 0
        self.y = 0
        self.target_latitude_min = 0
        self.target_latitude_max = 0
        self.target_longtitude_min = 0
        self.target_longtitude_max = 0

    def setting(self,lat,lon):
        lat = lat
        lon = lon
        shape = input('shape of safe area : ')
        if shape == 'rec' or shape == 'circle' or shape == 'square':
            print(shape, 'is selected')
            if shape == 'rec':
                print('decide x and y')
                x = input('x : ')
                y = input('y : ')
                print('the range is',x,'x',y)
                degx, degy = dist_deg_trans(x,y)
                self.target_latitude_min = lat - degy/2
                self.target_latitude_max = lat + degy/2
                self.target_longtitude_min = lon - degx/2
                self.target_longtitude_max = lon + degx/2
                
            elif shape == 'circle':
                print('decide radius')
                r = input("r :")
                print('the range is',r)
            else:
                print('decide the length of one side :')
                x = input('x :')
                print('the range is',x,'x',x)
                degx, degy = dist_deg_trans(x,x)
                self.target_latitude_min = lat - degx/2
                self.target_latitude_max = lat + degx/2
                self.target_longtitude_min = lon - degy/2
                self.target_longtitude_max = lon + degy/2
        else:
            print("shoude be rec or circle or square")

    def dist_deg_trans(x,y):
        degx = x/6371000*180/np.pi*60          # x , earth radius , radian to degree , degree to minute
        degy = y/6371000*180/np.pi*60          # d = degree, m = minute , latitude and longitude = ddmm.mmmm format
        return degx,degy


if __name__ == "__main__":
    rospy.init_node("GPS_reading", anonymous = True)
    rospy.loginfo("GPS_STARTING")
    try :
        gps_function = gps()
        time.sleep(1)
        while(gps.latitude == 0 or longitue == 0):
            gps_function.gps_reading()
        safe_area = safe_area()
        safe_area.setting(gps_function.latitude,gps_function.longtitude)

        gps_function.init_lat = gps_function.latitude
        gps_function.init_long = gps_function.longtitude
        gps_function.target_latitude_max = safe_area.target_latitude_max
        gps_function.target_latitude_min = safe_area.target_latitude_min
        gps_function.target_longtitude_max = safe_area.target_longtitude_max
        gps_function.target_longtitude_min = safe_area.target_longtitude_min
        gps_function.safe_area_shape = safe_area.shape
        gps_function.target_radius = safe_area.r

        while not rospy.is_shutdown():
            gps_function.gps_reading()
        except rospy.ROSInterruptException:
            print "ROS Terminated"
            pass