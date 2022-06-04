import serial
from bearing import Bearing
from distance import Distance
import time
import string
import sys
import pynmea2
from berryIMU import Heading


Kansas = [39.099912, -94.581213]

x = Bearing()
y = Heading()

while True:
	port="/dev/ttyAMA0"
	ser=serial.Serial(port, baudrate=9600, timeout=0.5)
	dataout = pynmea2.NMEAStreamReader()
	newdata=ser.readline()

	if(sys.version_info[0] == 3):
			newdata = newdata.decode("utf-8","ignore")
	if newdata[0:6] == "$GPRMC":
		newmsg=pynmea2.parse(newdata)
		lat=newmsg.latitude
		lng=newmsg.longitude
		#gps = "Latitude=" + str(lat) + "and Longitude=" + str(lng)

		current_loc  = [float(lat), float(lng)]
		bearing = x.find_bearing(current_loc, [39.099912, -94.581213])
		heading = y.get_heading()

		print("Bearing: " + str(bearing) + " Heading: " + str(heading))
