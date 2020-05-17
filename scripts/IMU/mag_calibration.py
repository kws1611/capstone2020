#!/usr/bin/env python

import rospy
import smbus
import matplotlib.pyplot as plt
from MPU9250 import MPU

if __name__ == "__main__":
    rospy.init_node("mag_calibration")
    
    mag_cali = MPU(500, 4, 16)

    raw_data, calibrated_data = mag_cali.calibrateMag(500)

    # X-Y plane
    plt.figure(num=1)
    plt.title("X-Y")
    plt.xlabel("X-axis")
    plt.xlabel("Y-axis")
    plt.scatter(calibrated_data[0], calibrated_data[1])

    # Y-Z plane
    plt.figure(num=2)
    plt.title("Y-Z")
    plt.xlabel("Y-axis")
    plt.xlabel("Z-axis")
    plt.scatter(calibrated_data[1], calibrated_data[2])

    # Z-X plane
    plt.figure(num=3)
    plt.title("Z-X")
    plt.xlabel("Z-axis")
    plt.xlabel("X-axis")
    plt.scatter(calibrated_data[2], calibrated_data[0])

    plt.show()

    rospy.set_param("/mag_bias", [mag_cali.magXbias, mag_cali.magYbias, mag_cali.magZbias])
    rospy.set_param("/mag_scale", [mag_cali.magXscale, mag_cali.magYscale, mag_cali.magZscale])