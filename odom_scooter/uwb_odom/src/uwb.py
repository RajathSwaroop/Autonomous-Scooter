#!/usr/bin/env python
import serial
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import sys
import string

def sub():
	p2p1 = [0]*2
	p3p1 = [0]*2	
	pub = rospy.Publisher('uwb_odom',String,queue_size=10)
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10)
	port = '/dev/ttyACM0'
	try:
		ser = serial.Serial(port=port,baudrate=115200,timeout=1)
	except serial.serialutil.SerialException:
		rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    
    		sys.exit(0)
	while True:
		line = ser.readline()	
		#rospy.loginfo(line)
		words = string.split(line,";")
		#rospy.loginfo(words)
		d1 = int(words[0])

		d2 = int(words[1])
	
		d3 = int(words[2])
	
		rospy.loginfo(d1)
		rospy.loginfo(d2)
		rospy.loginfo(d3)

		rospy.loginfo("\n")

		P1 = [0]*2
		P2 = [0]*2
		P3 = [0]*2
		P1[0] = 0
		P1[1] = 8
		P2[0] = 0
		P2[1] = 4
		P3[0] = 1;
		P3[1] = 2;

	
		p2p1 = (P2[0]-P1[0])*(P2[0]-P1[0]) + (P2[1]-P1[1])*(P2[1]-P1[1]);
		p2d = math.sqrt(p2p1);
		ex  = [0]*2
	#	//Calculates i and j by rotating the plane of axis -> (ival,jval)
		ex[0] = (P2[0]-P1[0])/p2d;
		ex[1] = (P2[1]-P1[1])/p2d;
		p3p1[0] = P3[0] - P1[0];
		p3p1[1] = P3[1] - P1[1];

		ival = (ex[0] * p3p1[0]) + (ex[1] * p3p1[1]);
		ey = [0]*2
		t0 = P3[0] - P1[0] - ex[0] * ival;
		t1 = P3[1] - P1[1] - ex[1] * ival;
		p3p1i = t0*t0 + t1*t1;
		ey[0] = t0/math.sqrt(p3p1i);
		ey[1] = t1/math.sqrt(p3p1i);

		jval = ey[0]*p3p1[0] + ey[1]*p3p1[1];

	#	//Computes the calculation for x and y of tag using the new vectors: (0,0), (d,0), and (i,j)
		xval = (d1*d1 - d2*d2 + p2d*p2d)/(2*p2d);
		yval = (d1*d1 - d3*d3 + ival*ival + jval*jval)/(2*jval) - (ival*xval)/jval;

	#	//Undo all transformations. Offset back (P1) and rotate back (ex,ey)
		tagx = P1[0] + ex[0]*xval + ey[0]*yval;
		tagy = P1[1] + ex[1]*xval + ey[1]*yval;

if __name__ == '__main__':

	try:
		sub()
	except rospy.ROSInterruptException:
		pass
