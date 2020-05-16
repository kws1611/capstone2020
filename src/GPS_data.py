
import serial
import pynmea2

def parseGPS(str):
    if str.find('GGA') >0:
        msg=pynmea2.parse(str)
        print(msg)    
        print(msg.lat)
        print(msg.lon)
        print(msg.altitude)

serialPort=serial.Serial("/dev/serial0",9600,timeout=0.5)

while True:
    str=serialPort.readline()
    parseGPS(str)
