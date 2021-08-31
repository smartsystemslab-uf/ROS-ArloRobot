#!/usr/bin/env python

# ROS Dependencies
import math
from math import sin, cos, pi
import re

import rospy
import tf
import numpy as np

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Core Dependencies
import serial
import time
import mmap

# Use relay for automatic error reset
relay_support = True

global currentLeftWheelSpeed
global currentRightWheelSpeed
global previousLeftWheelSpeed
global previousRightWheelSpeed

def initialize_motor_controller_serial_reader():
 serial_reader = serial.Serial('/dev/ttyUL2', 115200, timeout=0.05)
 return serial_reader


def initialize_motor_controller_serial_writer():
 # Connect first as default baud rate
 serial_writer = serial.Serial('/dev/ttyPS3', 19200)
 serial_writer.write('baud 115200\r'.encode())
 serial_writer.close()

 # Connect at the desired baud rate
 serial_writer = serial.Serial('/dev/ttyPS3', 115200)
 #ser.baudrate = 115200

 # Clear invalid baud rate command if baud rate already set
 serial_writer.write('\r'.encode())
 serial_writer.write('\r'.encode())

 # Set motor controller parameters
 serial_writer.write('txpin ch2\r'.encode()) # Move tx to ch2 pin
 time.sleep(0.01)

 return serial_writer


# This function is special for one of the Arlo robots with
#    the feather/relay stacked and acting as a UART device
def enable_motor_controller_power_feather_relay_stack():
 serial_relay = serial.Serial('/dev/ttyUSB0', 115200)
 serial_relay.write('ON'.encode())
 serial_relay.close()
 time.sleep(5)

def disable_motor_controller_power_feather_relay_stack():
 serial_relay = serial.Serial('/dev/ttyUSB0', 115200)
 serial_relay.write('OFF'.encode())
 serial_relay.close()
 time.sleep(1)



def enable_motor_controller_power_gpio_relay():
 # Open the file descriptor for the GPIO IP
 f = open('/dev/uio3', 'r+b')

 # Map a single byte to memory (this will be the byte used by the GPIO pins)
 #  Bit 7-2 = NC, Bit 1 = GPIO Output 1, Bit 0 = GPIO Output 0
 m = mmap.mmap(f.fileno(), 1)
 m.seek(0)

 # GPIO Output 1 (Relay) High
 m.write_byte('2')

 m.close()
 f.close()
 time.sleep(5)


def disable_motor_controller_power_gpio_relay():
 # Open the file descriptor for the GPIO IP
 f = open('/dev/uio3', 'r+b')

 # Map a single byte to memory (this will be the byte used by the GPIO pins)
 #  Bit 7-2 = NC, Bit 1 = GPIO Output 1, Bit 0 = GPIO Output 0
 m = mmap.mmap(f.fileno(), 1)
 m.seek(0)

 # GPIO Output 1 (Relay) Low
 m.write_byte('0')

 m.close()
 f.close()
 time.sleep(1)


def velocity_callback(data):
 global currentLeftWheelSpeed
 global currentRightWheelSpeed
 global previousLeftWheelSpeed
 global previousRightWheelSpeed

 #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
 if robot_in_error:
  return

 currentLeftWheelSpeed = 0
 currentRightWheelSpeed = 0

 if data.linear.x:
  currentLeftWheelSpeed = encoder_postions_per_meter * data.linear.x
  currentRightWheelSpeed = encoder_postions_per_meter * data.linear.x

 if data.angular.z > 0:
  currentRightWheelSpeed += encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
  currentLeftWheelSpeed -= encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
 elif data.angular.z < 0:
  currentRightWheelSpeed += encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters
  currentLeftWheelSpeed -= encoder_postions_per_meter * data.angular.z * wheelbase_radius_meters

 if previousLeftWheelSpeed != currentLeftWheelSpeed or previousRightWheelSpeed != currentRightWheelSpeed:
  serial_writer.write(('gospd ' + str(int(currentLeftWheelSpeed)) + ' ' + str(int(currentRightWheelSpeed)) + '\r').encode())
  print('GOSPD' + str(int(currentLeftWheelSpeed)) + ' ' + str(int(currentRightWheelSpeed)))

 previousLeftWheelSpeed = currentLeftWheelSpeed
 previousRightWheelSpeed = currentRightWheelSpeed



if __name__ == '__main__':
 # Init ROS node
 ros_ns = rospy.get_namespace()
 rospy.init_node('motor_controller', anonymous=False)
 rospy.Subscriber('/' + ros_ns + '/cmd_vel', Twist, velocity_callback)

 robot_saved_state = open('/var/log/ROS/dhb10-controller/saved-robot-state','w+')

 if relay_support:

  # Disable the motor controller power in case it is already on
  # disable_motor_controller_power()
  disable_motor_controller_power_gpio_relay()

  # Enable the motor controller power
  # enable_motor_controller_power()
  enable_motor_controller_power_gpio_relay()


 # Init the motor controller and get the serial writer
 serial_writer = initialize_motor_controller_serial_writer()

 # Init the serial reader
 serial_reader = initialize_motor_controller_serial_reader()



 odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
 clone_pub = rospy.Publisher("clone_odom", Odometry, queue_size=50)
 tf_broadcaster = tf.TransformBroadcaster()

 # Statics
 wheelbase_radius_meters = 0.19685
 wheelbase_diameter_meters = wheelbase_radius_meters * 2
 encoder_postions_per_meter = 300.7518

 # States
 # TODO - Use industrial messages/robot status here! http://docs.ros.org/kinetic/api/industrial_msgs/html/msg/RobotStatus.html
 robot_state = None
 robot_in_error = False



 # Tracked odometry
 #new
 robot_odom = np.array([('x', 0), ('y', 0), ('th', 0)], dtype=[('name', 'U10'), ('coord', 'i4')])
 #old
 #x = 0.0
 #y = 0.0
 #th = 0.0
 
 #new
 robot_vel = np.array([('vx', 0), ('vy', 0), ('vth', 0)], dtype=[('name', 'U10'), ('velocity', 'i4')]) 
 #old
 #vx = 0.0
 #vy = 0.0
 #vth = 0.0

 # Euler integrated pose defined in odometry frame
 #new
 timestep_plus_one = np.array([('x', 0), ('y', 0), ('th', 0)], dtype=[('name', 'U10'), ('coord', 'i4')]) 
 #old
 #x_timestep_plus_one = 0.0
 #y_timestep_plus_one = 0.0
 #th_timestep_plus_one = 0.0

 # Postional/rotational offset from a saved pose state for when the motor controller is reset during a run
 x_pos_saved_state_offset = 0.0
 y_pos_saved_state_offset = 0.0
 y_pos_saved_state_offset = 0.0
 x_orient_saved_state_offset = 0.0
 y_orient_saved_state_offset = 0.0
 z_orient_saved_state_offset = 0.0
 w_orient_saved_state_offset = 0.0
 covar_saved_state_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

 # Encoder readings [Left, Right]
 #new
 wheel_encoder_pos_per_sec = np.array([('left', 0), ('right', 0)], dtype=[('side', 'U10'), ('value', 'i4')])
 #old
 #left_wheel_encoder_pos_per_sec = 0.0
 #right_wheel_encoder_pos_per_sec = 0.0


 # Velocity estimation [Left, Right]
 #new    
 wheel_velocity = np.array([('left', 0), ('right', 0)], dtype=[('side', 'U10'), ('velocity', 'i4')])
 #old
 #left_wheel_velocity = 0.0
 #right_wheel_velocity = 0.0

 #new [Vx meters, Vth meters, Vth radians]
 robot_meters_per_sec = np.array([('vx_meters', 0), ('vth_meters', 0), ('vth_rads', 0)], dtype=[('name', 'U10'), ('velocity', 'i4')])
 #old
 #vx_meters_per_sec = 0.0
 #vth_meters_per_sec = 0.0
 #vth_radians_per_sec = 0.0

 # Position estimations
 left_wheel_pos_meters = 0.0
 right_wheel_pos_meters = 0.0

 # Odometry frame [vx, vy, vth]
 #new
 odom_meters_per_sec = np.array([('vx_meters', 0), ('vy_meters', 0), ('vth_rads', 0)], dtype=[('name', 'U10'), ('velocity', 'i4')])
 #old
 #odom_vx_meters_per_sec = 0.0
 #odom_vy_meters_per_sec = 0.0
 #odom_vth_radians_per_sec = 0.0

 currentLeftWheelSpeed = 0
 currentRightWheelSpeed = 0
 previousLeftWheelSpeed = 0
 previousRightWheelSpeed = 0

 current_time = rospy.Time.now()
 last_time = rospy.Time.now()

 # refresh_rate = 10.0 # Hz
 # r = rospy.Rate(10.0)

 while not rospy.is_shutdown():

  serial_writer.write(('spd\r').encode())
  speed_response = serial_reader.read(100)

  heading_response = str(th)

  print (speed_response)

  if speed_response is '':
   print('MOTOR CONTROLLER INITIALIZATION ERROR')
   #exit()

   # Try closing the serial port

  elif 'motor' in speed_response or 'encoder' in speed_response or 'error' in speed_response:
   print('MOTOR CONTROLLER ERROR')

   print('RESETTING MOTOR CONTROLLER')

   robot_in_error = True

   if relay_support:

    # disable_motor_controller_power()
    # enable_motor_controller_power()
    disable_motor_controller_power_gpio_relay()
    enable_motor_controller_power_gpio_relay()

    serial_writer.close()
    serial_reader.close()


    # Init the motor controller and get the serial writer
    serial_writer = initialize_motor_controller_serial_writer()

    # Init the serial reader
    serial_reader = initialize_motor_controller_serial_reader()

    # Close the state file writer
    # robot_saved_state.close()

    # Get the last reported pose so as to not throw off any running localization nodes
    # -Odom message format example
    #	pose:
    #	  position:
    #	    x: 0.0
    #	    y: 0.0
    #	    z: 0.0
    #	  orientation:
    #	    x: 0.0
    #	    y: 0.0
    #	    z: 0.0
    #	    w: 1.0
    #	covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0]
    # robot_saved_state = open('/home/ubuntu/ROS/arlo_ws/src/arlo-motor-controller-interface/saved-robot-state')
    robot_saved_state.seek(0)
    robot_saved_state.readline()
    robot_saved_state.readline()
    x_pos_saved_state_offset = float(robot_saved_state.readline().split()[1])
    y_pos_saved_state_offset = float(robot_saved_state.readline().split()[1])
    z_pos_saved_state_offset = float(robot_saved_state.readline().split()[1])
    robot_saved_state.readline()
    x_orient_saved_state_offset = float(robot_saved_state.readline().split()[1])
    y_orient_saved_state_offset = float(robot_saved_state.readline().split()[1])
    z_orient_saved_state_offset = float(robot_saved_state.readline().split()[1])
    w_orient_saved_state_offset = float(robot_saved_state.readline().split()[1])
    #covar_saved_state_offset = eval(robot_saved_state.readline().split(': ')[1])

    print(str(x_pos_saved_state_offset))
    print(str(y_pos_saved_state_offset))
    print(str(z_pos_saved_state_offset))
    print(str(x_orient_saved_state_offset))
    print(str(y_orient_saved_state_offset))
    print(str(z_orient_saved_state_offset))
    print(str(w_orient_saved_state_offset))
    #print str(covar_saved_state_offset)

   # Reopen the state file writer
   # robot_saved_state = open('/home/ubuntu/ROS/arlo_ws/src/arlo-motor-controller-interface/saved-robot-state','w+')

  elif speed_response is not '':

   print('Speed: ' + speed_response + '   Heading: ' + heading_response)

   if re.match(r'-*\d{1,3}\s-*\d{1,3}', speed_response):
    robot_in_error = False
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    #new
    wheel_encoder_pos_per_sec[0] = int(speed_response.split()[0])
    wheel_encoder_pos_per_sec[1] = int(speed_response.split()[1])
    #old
    #left_wheel_encoder_pos_per_sec = int(speed_response.split()[1])
    #right_wheel_encoder_pos_per_sec = int(speed_response.split()[1])

    #new
    wheel_velocity = wheel_encoder_pos_per_sec / encoder_postions_per_meter
    #old
    #left_wheel_velocity = left_wheel_encoder_pos_per_sec / encoder_postions_per_meter
    #right_wheel_velocity = right_wheel_encoder_pos_per_sec / encoder_postions_per_meter

    #new
    robot_meters_per_sec['velocity'][0] = (wheel_velocity['velocity'][0] + wheel_velocity['velocity'][1]) / 2
    robot_meters_per_sec['velocity'][2] = (wheel_velocity['velocity'][0] - wheel_velocity['velocity'][1]) / wheelbase_diameter_meters
    #old
    #vx_meters_per_sec = (left_wheel_velocity + right_wheel_velocity) / 2
    #vth_radians_per_sec = (right_wheel_velocity - left_wheel_velocity) / wheelbase_diameter_meters

    # th = float(int(heading_response) * math.pi / 180) # $$$ idk what this is 
    #new
    odom_meters_per_sec['velocity'][0] = robot_meters_per_sec['velocity'][0] * np.cos(robot_odom['coord'][2])
    odom_meters_per_sec['velocity'][1] = robot_meters_per_sec['velocity'][1] * np.sin(robot_odom['coord'][2])
    odom_meters_per_sec['velocity'][2] = robot_meters_per_sec['velocity'][2]
    #old
    #odom_vx_meters_per_sec = vx_meters_per_sec * math.cos(th) 
    #odom_vy_meters_per_sec = vx_meters_per_sec * math.sin(th) 
    #odom_vth_radians_per_sec = vth_radians_per_sec

    #new
    timestep_plus_one['coord'][0] = robot_odom['coord'][0] + (odom_meters_per_sec['velocity'][0] * dt)
    timestep_plus_one['coord'][1] = robot_odom['coord'][1] + (odom_meters_per_sec['velocity'][1] * dt)
    timestep_plus_one['coord'][2] = robot_odom['coord'][2] + (odom_meters_per_sec['velocity'][2] * dt)
    #old
    #x_timestep_plus_one = x + (odom_vx_meters_per_sec * dt)
    #y_timestep_plus_one = y + (odom_vy_meters_per_sec * dt)
    #th_timestep_plus_one = th + (odom_vth_radians_per_sec * dt)
   
   # Add any saved state offset, if any, to the odometry from motor controller readings
   #new
   timestep_plus_one['coord'][0] += x_pos_saved_state_offset
   timestep_plus_one['coord'][1] += y_pos_saved_state_offset
   timestep_plus_one['coord'][2] += z_orient_saved_state_offset
   #old
   #x_timestep_plus_one += x_pos_saved_state_offset
   #y_timestep_plus_one += y_pos_saved_state_offset
   #th_timestep_plus_one += z_orient_saved_state_offset

   print(str(timestep_plus_one['coord'][0]) + ' ' + str(timestep_plus_one['coord'][1]) + ' ' + str(timestep_plus_one['coord'][2]))

   # since all odometry is 6DOF we'll need a quaternion created from yaw
   odom_quat = tf.transformations.quaternion_from_euler(0, 0, timestep_plus_one['coord'][2])

   # first, we'll publish the transform over tf
   tf_broadcaster.sendTransform(
    (timestep_plus_one['coord'][0], timestep_plus_one['coord'][1], 0.),
    odom_quat,
    current_time,
    "base_link",
    "odom"
   )

   # next, we'll publish the odometry message over ROS
   odom = Odometry()
   odom.header.stamp = current_time
   odom.header.frame_id = "odom"

   # Set the position
   odom.pose.pose = Pose(Point(timestep_plus_one['coord'][0], timestep_plus_one['coord'][1], 0.), Quaternion(*odom_quat))

   # Set the velocity
   odom.child_frame_id = "base_link"
   odom.twist.twist = Twist(Vector3(robot_meters_per_sec['velocity'][0], 0, 0), Vector3(0, 0, robot_meters_per_sec['velocity'][2]))

   # Publish the message
   odom_pub.publish(odom)
   robot_saved_state.seek(0)
   robot_saved_state.write('\rSpeed: ' + str(odom.pose))


   last_time = current_time
   #new
   robot_odom['coord'][0] = timestep_plus_one['coord'][0]
   robot_odom['coord'][1] = timestep_plus_one['coord'][1]
   robot_odom['coord'][2] = timestep_plus_one['coord'][2]
   #old
   #x = x_timestep_plus_one
   #y = y_timestep_plus_one
   #th = th_timestep_plus_one

   #new
   #old
   #if th >= 2*math.pi:
   # th = th - 2*math.pi

   #if th < 0:
   # th = 2*math.pi - th

  rospy.sleep(0.001)