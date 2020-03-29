#!/usr/bin/env python

import time
import pigpio
import rospy
import rospy
import numpy as np
from rise_control.msg import ppm_msg
from std_msgs.msg import String


class X:
    WAVES = 8
    def __init__(self, pi, frame_ms=33):
        self.pi = pi
        self.gpio = rospy.get_param("output/gpio")
        self.rate = rospy.Rate(1000)
        self.GAP = 300

        self.frame_ms = frame_ms
        self._frame_us = int(frame_ms * 1000)
        self._frame_secs = frame_ms / 1000.0
        self.check_count = 0
        self.ch1 = 1500
        self.ch2 = 1000
        self.ch3 = 1000
        self.ch4 = 1000
        self.ch5 = 1000
        self.ch6 = 1500
        self.ch7 = 1000
        self.ch8 = 1000

        self.channels = rospy.get_param("channel_number")


        self._widths = [1000] * self.channels  # set each channel to minimum pulse width
        print(self._widths)

        self._wid = [None] * self.WAVES
        self._next_wid = 0

        pi.write(gpio, pigpio.LOW)

        self._update_time = time.time()
        rospy.Subscriber("/control_signal", ppm_msg, self.ppm_cb)
        self.ppm_output_pub = rospy.Publisher("/output_ppm", ppm_msg, queue_size=1)


    def ppm_cb(self, msg):
        self.ppm_input_msg = msg
        self.ch1 = int(self.ppm_input_msg.channel_1)
        self.ch2 = int(self.ppm_input_msg.channel_2)
        self.ch3 = int(self.ppm_input_msg.channel_3)
        self.ch4 = int(self.ppm_input_msg.channel_4)
        self.ch5 = int(self.ppm_input_msg.channel_5)
        self.ch6 = int(self.ppm_input_msg.channel_6)
        self.ch7 = int(self.ppm_input_msg.channel_7)
        self.ch8 = int(self.ppm_input_msg.channel_8)


    def _update(self):
        wf = []
        micros = 0
        for i in self._widths:
            wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))
            wf.append(pigpio.pulse(0, 1 << self.gpio, i - self.GAP))
            micros += i
        # off for the remaining frame period
        wf.append(pigpio.pulse(1 << self.gpio, 0, self.GAP))
        micros += self.GAP
        wf.append(pigpio.pulse(0, 1 << self.gpio, self._frame_us - micros))
        self.pi.wave_add_generic(wf)
        wid = self.pi.wave_create()

        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
        self._wid[self._next_wid] = wid

        self._next_wid += 1
        if self._next_wid >= self.WAVES:
            self._next_wid = 0

        remaining = self._update_time + self._frame_secs - time.time()
        if remaining > 0:
            time.sleep(remaining)
        self._update_time = time.time()

        wid = self._wid[self._next_wid]
        if wid is not None:
            self.pi.wave_delete(wid)
            self._wid[self._next_wid] = None

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)

    def sending_process(self):
        #rospy.Subscriber("/input_ppm", ppm_msg, self.ppm_cb)
        self.sending_topic = ppm_msg()
        chan_1 = self.ch1
        chan_2 = self.ch2
        chan_3 = self.ch3
        chan_4 = self.ch4
        chan_5 = self.ch5
        chan_6 = self.ch6
        chan_7 = self.ch7
        chan_8 = self.ch8
        self.check_count += 1

        self._widths[0] = self.ch1 + 500
        self._widths[1] = self.ch2 + 500
        self._widths[2] = self.ch3 + 500
        self._widths[3] = self.ch4 + 500
        self._widths[4] = 1500
        self._widths[5] = 1500
        self._widths[6] = 1500
        self._widths[7] = 1500
        self._update()

        print(chan_1 + 500, chan_2 + 500, chan_3 + 500, chan_4 + 500, chan_5, chan_6 + 500, chan_7 + 500, chan_8 + 500)
        self.rate.sleep()
if __name__ == "__main__":
    rospy.init_node("ppm_sending", anonymous=True)
    rospy.loginfo("ppm_generating start")
    pi = pigpio.pi()

    try:
        if not pi.connected:
            exit(0)
        pi.wave_tx_stop()  # Start with a clean slate.
        ppm = X(pi, frame_ms=20)
        time.sleep(2)

        while not rospy.is_shutdown():

            ppm.sending_process()
        ppm.cancel()
        pi.stop()
    except rospy.ROSInterruptException:
        print
        "ROS terminated"
        pass
