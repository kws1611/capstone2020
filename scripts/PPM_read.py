#!/usr/bin/python

import rospy
from capstone2020.msg import Ppm
import Jetson.GPIO as GPIO
import time

class PPM_read:
    def __init__(self):
        self.in_pin = 13 

        self.start = None
        self.tic = None
        self.channel = None

        self.frame = 20000          # frame length (microsecons)
        self.blank_time = 3000      # (microsecons)
        self.max_ch = 8 

        self.raw_PPM = Ppm()
        self.PPM = Ppm()
        self.input_ppm_pub = rospy.Publisher('/input_ppm', Ppm, queue_size= 1)

        GPIO.setup(self.in_pin, GPIO.IN)
        GPIO.add_event_detect(self.in_pin, GPIO.RISING, callback=self.cb)

    def cb(self, in_pin):
        if self.tic == None:
            self.tic = time.time()

            return
        else:
            diff = (time.time() - self.tic) * 1e6  # microseconds
            self.tic = time.time()

        if diff > self.blank_time:
            self.start = True
            self.channel = 0
            
            return

        if self.start:
            self.raw_PPM.channel[self.channel] = diff
            self.channel += 1

            if self.channel == self.max_ch:
                for i in range(self.max_ch):
                    self.PPM.channel[i] = self.raw_PPM.channel[i]

    def signal_lost_detect(self):
        if self.tic == None:
            rospy.loginfo("Wait RC signal")

        elif (time.time() - self.tic) > self.frame:
            rospy.loginfo("WARNING: RC Signal Lost")

            self.PPM.channel = None

    def publishing(self):
        #self.PPM.channel.header.stamp = rospy.Time.now()
        self.input_ppm_pub.publish(self.PPM)

if __name__ == '__main__':
    rospy.init_node('PPM_read')

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    reciever = PPM_read()
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        reciever.publishing()

        rate.sleep()
