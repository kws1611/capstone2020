#!/usr/bin/python

import rospy
from capstone2020.msg import Ppm
import Jetson.GPIO as GPIO
import time

class PPM_write:
    def __init__(self):
        self.out_pin = 11

        self.frame = 20000 * 1e-6
        self.pulse_time = 100 * 1e-6

        self.cnt = 0
        self.delay_data = [0 for i in range(90)]
        self.avg_delay = 0
        self.sample_num = 90

        self.input_ppm = None

        rospy.Subscriber('/output_ppm', Ppm, self.ppm_cb)

        GPIO.setup(self.out_pin, GPIO.OUT, initial=GPIO.LOW)

    def ppm_cb(self, msg):
        self.input_ppm = msg.channel

    def pulse(self):
        GPIO.output(self.out_pin, GPIO.HIGH)

        start = time.time()
        time.sleep(self.pulse_time)

        GPIO.output(self.out_pin, GPIO.LOW)
        delay = (time.time() - start) - self.pulse_time

        self.delay_data[self.cnt] = delay

        if self.cnt == self.sample_num - 1:
            self.cnt = 0
            self.avg_delay = sum(self.delay_data) / self.sample_num
        else:
            self.cnt += 1
            
    def write(self, input_ppm):
        if input_ppm == None:
            return
        
        channel = 0

        while channel < 8:
            start = time.time()
            self.pulse()
            time.sleep(input_ppm[channel]*1e-6 - self.pulse_time - 2*self.avg_delay)

            channel += 1
            
            if(channel == 8):
                self.pulse()
                remain = self.frame - time.time() + start
                time.sleep(remain - 2*self.avg_delay)

if __name__ == '__main__':
    rospy.init_node('PPM_write')

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    transmitter = PPM_write()
    
    while not rospy.is_shutdown():
        transmitter.write([2000, 1500, 1500, 1500, 1000, 1000, 1000, 1000])
        #transmitter.write(self.input_ppm)