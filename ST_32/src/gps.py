#!/usr/bin/env python

import rospy
import serial
import string
from std_msgs.msg import String
import time
import sys
from math import sin, cos, sqrt, atan2, radians

i = sys.argv

port='/dev/ttyACM' + i[1]
R = 6373.0
final_lat = 37.3345
final_lng = -121.8898
try:
	ser = serial.Serial(port=port, baudrate=9600, timeout=1)
except serial.serialutil.SerialException:
	rospy.logerr("GPS not found at port "+port + ". Did you specify the correct port in the launch file?")
	sys.exit(0)

def callback(data):
	read_imu = data.data
	new_data = string.split(read_imu,":")
	if(len(new_data)>4):
		 
 
def sub():
	pub = rospy.Publisher('Nav_data',String,queue_size=10)
	rospy.init_node('outdoor_navigation',anonymous=True)
	rate = rospy.Rate(10)
	i = sys.argv
		
	while True:
		rospy.Subscriber("IMU_data", String, callback)
		line = ser.readline()				#read incoming GPS data
		print(line)
		words = string.split(line,";")			#split words to read heading, latitude and longitude
		if(len(words)>2):
			next = string.split(words[0],":")
			heading = float(next[1])		#reading heading
			next = string.split(words[1],":")
			lat = float(next[1])			#reading latitude
			next = string.split(words[2],":")
			lng = float(next[1])			#reading longitude
		
		print("heading is:{}".format(heading))		
		print("lat is:{}".format(lat))
		print("lng is:{}".format(lng))
		current_lat = radians(lat)			#convert current latitude to radians
		current_lng = radians(lng)			#convert current longitude to radians
		dest_lat = radians(final_lat)			#convert destination latitude to radians
		dest_lng = radians(final_lng)			#convert destination longitude to radians
		dlat = dest_lat - current_lat			#getting the difference in latitudes
		dlng = dest_lng - current_lng			#getting the difference in longitudes
	
		a = float(sin(dlat / 2)**2 + cos(current_lat) * cos(dest_lat) * sin(dlng / 2)**2)
		c = float(2 * atan2(sqrt(a), sqrt(1 - a)))
#		print(a)
#		print(c)
		distance = float(R*c)				#compute the distance to travel between the two co-ordinates
		print(distance)
		bearing = atan2(cos(current_lat)*sin(current_lng)-sin(current_lat)*cos(dest_lat)*cos(dest_lng-current_lng),sin(dest_lng-current_lng)*cos(dest_lat))	#bearing angle
		print(bearing)
		angle_to_turn = bearing - heading		#angle to turn
		print(angle_to_turn)
		msg = "distance to travel is:"+str(distance)+", bearing angle is:"+str(bearing)
		pub.publish(msg)				#publish the data to drive

if __name__ == '__main__':

	try:
		sub()
	except rospy.ROSInterruptException:
		pass

