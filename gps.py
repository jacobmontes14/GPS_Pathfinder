import serial
from bearing import Bearing
from distance import Distance
import time
import string
import sys
import pynmea2
from berryIMU import Heading

class Gps:
    def __init__(self):
        self.port="/dev/ttyAMA0"

    def get_coord(self):
        ser=serial.Serial(self.port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        newdata=ser.readline()

        if(sys.version_info[0] == 3):
                newdata = newdata.decode("utf-8","ignore")
        if newdata[0:6] == "$GPRMC":
            newmsg=pynmea2.parse(newdata)
            lat=newmsg.latitude
            lng=newmsg.longitude
            gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)
            return gps
