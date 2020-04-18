#!/usr/bin/env python

import time
import math
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String

class PWM_read:
    def __init__(self, pi):
        self.pi = pi
        self.gpio = rospy.get_param("input/gpio")
        self._high_tick = None
        self._p = None
        self._hp = None
        self.correcting_channel = 0
        self.channel_number = [0,0,0,0,0,0,0,0]
        self.count = 0
        self.basic_ppm_signal_count = 0
        self.ch = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ch_final = [0,0,0,0,0,0,0,0,0]
        self.rate = rospy.Rate(50)
        self.first_switch = True

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)
        self.input_pub = rospy.Publisher("/input_ppm",ppm_msg,queue_size=1)

    def _cbf(self, gpio, level, tick):
        input_channel = ppm_msg()
        if level == 1:
            if self._high_tick is not None:
                self._p = pigpio.tickDiff(self._high_tick, tick)
            self._high_tick = tick
        elif level == 0:
            if self._high_tick is not None:
                    self._hp = pigpio.tickDiff(self._high_tick, tick)
        if (self._p is not None) and (self._hp is not None):
            self.ch[self.count] = self._hp
            self.count += 1
            if self.count == 18:
                self.count = 0
    def reading_process(self):
        #print("reading_proce")
        read_message = ppm_msg()
        if self.first_switch:
            if self.ch[self.count] > 7000:
                self.basic_ppm_signal_count = self.count
                self.first_switch = False

            for i in range(0,7):
                if self.basic_ppm_signal_count + 2 + 2*i > 17:
                    self.channel_number[i] = self.basic_ppm_signal_count + 2 + 2*i - 18
                else :
                    self.channel_number[i] = self.basic_ppm_signal_count + 2 + 2*i
        
        self.ch_final = [self.ch[self.channel_number[0]], self.ch[self.channel_number[1]], self.ch[self.channel_number[2]], self.ch[self.channel_number[3]], self.ch[self.channel_number[4]], self.ch[self.channel_number[5]], self.ch[self.channel_number[6]],self.ch[self.channel_number[7]]]
        read_message.channel_1 = self.ch_final[0]
        read_message.channel_2 = self.ch_final[1]
        read_message.channel_3 = self.ch_final[2]
        read_message.channel_4 = self.ch_final[3]
        read_message.channel_5 = self.ch_final[4]
        read_message.channel_6 = self.ch_final[5]
        read_message.channel_7 = self.ch_final[6]
        read_message.channel_8 = self.ch_final[7]
        read_message.header.stamp = rospy.Time.now()
        self.input_pub.publish(read_message)
        self.rate.sleep()
    
    def cancel(self):
        self._cb.cancel()

if __name__ == "__main__":
    rospy.init_node("ppm_reading", anonymous=True)
    rospy.loginfo("ppm_reading start")
    pi = pigpio.pi()    

    try:
        p1 = PWM_read(pi)
        time.sleep(1)
        
        while not rospy.is_shutdown():
            p1.reading_process()
        
    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass
    pi.stop()
