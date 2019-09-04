#!/usr/bin/env python

# Simple script for listening for control (Twist) commands via UDP for a differential-drive
#	robot. Generated twist commands are forwarded to the /cmd_vel topic.

import socket

import rospy
from geometry_msgs.msg import Twist, Vector3


SERVER_IP = '10.0.0.150'
SERVER_PORT = 8081

if __name__ == '__main__':

	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # socket.SOCK_DGRAM tells the server to listend fo UDP packets
	serverSocket.bind((SERVER_IP, SERVER_PORT))

	rospy.init_node('udp-control-server', anonymous=False)
	vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

	cmd_vel = Twist()
	print "UDP control server is listening..."
	while True:
		data, source_address = serverSocket.recvfrom(1024)
		#print "Message: " + data

		#	The expected messages are in the form of:
		#		LinearVelocity_x (m/s) RotationalVelocity_z (rad/s)
		cmd_vel = Twist(Vector3(float(data.split()[0]), 0, 0), Vector3(0, 0, float(data.split()[1])))

		vel_pub.publish(cmd_vel)
