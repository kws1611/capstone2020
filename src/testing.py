#!/usr/bin/env python

import serial
import binascii
import time


if __name__ == '__main__':
    s=serial.Serial("/dev/ttyS0",100000,serial.EIGHTBITS,serial.PARITY_EVEN, serial.STOPBITS_TWO)
    s.timeout = None
    t = ''
    c = 0
    
    
    while(1):
        alpha = s.read()
        print (alpha)
 
