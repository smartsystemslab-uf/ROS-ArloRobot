#!/usr/bin/env python

import socket

import rospy
from geometry_msgs.msg import Twist, Vector3


SERVER_IP = '10.0.0.150'
SERVER_PORT = 8081

if __name__ == '__main__':

 serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 serverSocket.bind((SERVER_IP, SERVER_PORT))
 
 rospy.init_node('arlo-ios-control-server', anonymous=False)
 vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

 cmd_vel = Twist()
 #cmd_vel.frame_id = "cmd_vel"
 print "IOS control server is listening..."
 while True:
  data, source_address = serverSocket.recvfrom(1024)
  #print "Message: " + data
  cmd_vel = Twist(Vector3(float(data.split()[0]), 0, 0), Vector3(0, 0, float(data.split()[1])))
  vel_pub.publish(cmd_vel)
