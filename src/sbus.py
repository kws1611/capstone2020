#!/usr/bin/env python

import serial
import time

class Sbus:

    def __init__(self):
        self.SBUS_FAILSAFE_INACTIVE = 0
        self.SBUS_FAILSAFE_ACTIVE   = 1
        self.SBUS_STARTBYTE         = 0x0f
        self.SBUS_ENDBYTE           = 0x00
        self.channels = []
        self.payload = []
        self.timeout = 0

    def merge_data(self):
        self.channels[0]  =  ((self.payload[0]    |self.payload[1] <<8)                     & 0x07FF)
        self.channels[1]  =  ((self.payload[1]>>3 |self.payload[2] <<5)                     & 0x07FF)
        self.channels[2]  =  ((self.payload[2]>>6 |self.payload[3] <<2 |self.payload[4]<<10)    & 0x07FF)
        self.channels[3]  =  ((self.payload[4]>>1 |self.payload[5] <<7)                     & 0x07FF)

    def parse(self, data):
        self.payload[0] = (data>>63) & 0x0000000000000000000FF
        self.payload[1] = (data>>51) & 0x0000000000000000000FF
        self.payload[2] = (data>>39) & 0x0000000000000000000FF
        self.payload[3] = (data>>27) & 0x0000000000000000000FF
        self.payload[4] = (data>>15) & 0x0000000000000000000FF
        self.payload[5] = (data>>3) & 0x0000000000000000000FF
    


if __name__ =="__main__":

    ser = serial.Serial ("/dev/ttyS0",baudrate = 100000, bytesize = serial.SIXBITS)    #Open named port, GPIO14(pin 8)(tx), GPIO15(pin 10)(RX)

                                                                                        #Set baud rate to 100000
    sbus = Sbus()
    startbit_mask = 0xFFF000000000000000000

    while(1):
        data = ser.read(15)                                         #Read 2byte from serial port to data
        if((data & startbit_mask)>>72 == 0b100001111100):           #start byte
                
            sbus.parse(data)
            sbus.merge_data()

            print(sbus.channels[0]," ", sbus.channels[1]," ", sbus.channels[2]," ", sbus.channels[3])
