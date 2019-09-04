#!/usr/bin/env python

# Script to implement some high level commands pretty quickly.
#	Actions are implemented as scripts in the action-scripts folder.

import os
import signal
import socket
import sys

def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)


def get_local_ip():
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	try:
		s.connect(('google.com', 80))
		ip = s.getsockname()[0]
		s.close()
	except:
		ip = 'N/A'
	return ip


def command_handler(command):
	print command
	if 'REBOOT' in command:
		print 'Command is: ' + command
		os.system("rosrun arlo-utilities reboot.sh")

	elif 'MOTORS ON' in command:
		print 'Command is: ' + command
		os.system("rosrun arlo-utilities motor-relay-on.sh &")
		# subprocess.call(['rosrun'], ['control-server'], ['motor-relay-on.sh'])

	elif 'MOTORS OFF' in command:
		print 'Command is: ' + command
		# subprocess.call(['rosrun'], ['control-server'], ['motor-relay-off.sh'])
		os.system("rosrun arlo-utilities motor-relay-off.sh &")

	elif 'MOTORS RESET' in command:
		print 'Command is: ' + command
		# subprocess.call(['rosrun'], ['control-server'], ['motor-relay-toggle.sh'])
		os.system("rosrun arlo-utilities motor-relay-toggle.sh &")

	elif 'LIDAR ON' in command:
		print 'Command is: ' + command
		os.system("rosservice call /start_motor &")

	elif 'LIDAR OFF' in command:
		print 'Command is: ' + command
		os.system("rosservice call /stop_motor &")

	elif 'FIGURE8' in command:
		print 'Command is: ' + command

	elif 'GREET' in command:
		print 'Command is: ' + command
		os.system("rosrun arlo-utilities greet.py &")

	elif 'GOTO' in command:

		if 'WP_1' in command or 'WP_One' in command:
			print 'Going to: WP_1'
		elif 'WP_2' in command or 'WP_Two' in command:
			print 'Going to: WP_2'
		elif 'WP_3' in command or 'WP_Three' in command:
			print 'Going to: WP_3'
		elif 'WP_4' in command or 'WP_Four' in command:
			print 'Going to: WP_4'
		elif 'WP_5' in command or 'WP_Five' in command:
			print 'Going to: WP_5'
		else:
			print 'Error: Unknown waypoint'

	else:
		print 'Error: Unknown command'


#print get_local_ip()
#TCP_IP = get_local_ip()
print '10.0.0.150'
TCP_IP = '10.0.0.150'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

signal.signal(signal.SIGINT, signal_handler)
print('Press Ctrl+C to exit...')

while 1:
	conn, addr = s.accept()
	print 'Connection address:', addr

	data = conn.recv(BUFFER_SIZE)
	if not data: break
	print "received data:", data
	conn.send(data)  # echo

	command_handler(data)

	conn.close()
