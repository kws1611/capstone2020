#!/usr/bin/env python

import rospy
import std_msgs.msg
from capstone2020.srv import setArea

def send_srv(longtitude, latitude, radius):
    rospy.wait_for_service('set_area')

    set_area = rospy.ServiceProxy("set_area", setArea)

    resp = set_area(longtitude, latitude, radius)

    if resp.result == True:
        print("OK")

    else:
        print("Fail")

def hook():
    print("Range should be more than 10m")

if __name__ == "__main__":
    minRange = 10

    print("Set Longtitude(Decimal[Deg]): ")
    longtitude = input()

    print("Set Latitude(Decimal[Deg]): ")
    latitude = input()

    print("Set Range(m): ")
    radius = input()

    if radius < minRange:
        rospy.on_shutdown(hook)

    else:
        send_srv(longtitude, latitude, radius)
