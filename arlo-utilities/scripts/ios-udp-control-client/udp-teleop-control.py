# Simple script for broadcasting control (Twist) commands via UDP for a differential-drive
#	robot. Generated twist commands will be forwarded to the /cmd_vel topic at the server.
# 	Control is performed by drawing the desired vector.
#
# NOTE: THIS SCRIPT WAS DEVELOPED IN AND REQUIRES PYTHONISTA ON IOS


from scene import *
import sound
import random
import math
import socket
import time

from objc_util import *
A = Action

UIImpactFeedbackGenerator = ObjCClass('UIImpactFeedbackGenerator')
UINotificationFeedbackGenerator = ObjCClass('UINotificationFeedbackGenerator')
UISelectionFeedbackGenerator = ObjCClass('UISelectionFeedbackGenerator')



class MyScene (Scene):
	def force_touch_handler(self, data):
		self.impactgeneratorheavy.impactOccurred()

	def setup(self):
		self.background_color = 'black'
		self.origin_sprite = None
		self.touch_sprite = None
		self.sprite = list()

		self.touchCount = 0
		self.storedTouches = list()


		self.linear_speed_increment = 0.05
		self.angular_speed_increment = 0.2
		self.touch_increment = 40

		self.deadzone_boundary_y = 50
		self.deadzone_boundary_x = 25


		self.current_touch_x = 0
		self.current_touch_y = 0

		self.touch_start_x = 0
		self.touch_start_y = 0


		self.linear_velocity = ''
		self.angular_velocity = ''
		self.previous_linear_velocity = ''
		self.previous_angular_velocity = ''

		self.impactgeneratorlight = UIImpactFeedbackGenerator.alloc().initWithStyle_(0)
		self.impactgeneratormedium = UIImpactFeedbackGenerator.alloc().initWithStyle_(1)
		self.impactgeneratorheavy = UIImpactFeedbackGenerator.alloc().initWithStyle_(2)
		self.notificationgenerator = UINotificationFeedbackGenerator.alloc().init()
		self.selectiongenerator = UISelectionFeedbackGenerator.alloc().init()

		self.Scene = Scene

		## SOCKETS ##

		self.servers = list()
		self.servers.append('10.0.0.150')
		self.serverPort = 8081

		# Create a UDP socket (source defined at send)
		self.udpClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	def did_change_size(self):
		pass

	def update(self):
		self.draw

	def touch_began(self, touch):

		if len(self.storedTouches) == 0:

			self.storedTouches.append(touch)

			self.touch_start_x, self.touch_start_y = touch.location

			# Touch origin sprite
			self.origin_sprite = SpriteNode('shp:Circle')
			self.origin_sprite.scale = 0.25
			self.origin_sprite.position = self.size / 2
			self.add_child(self.origin_sprite)
			x, y = touch.location
			move_action = Action.move_to(x, y, 0)
			self.origin_sprite.run_action(move_action)


			# Current touch sprite
			self.touch_sprite = SpriteNode('shp:Circle')
			self.touch_sprite.scale = 0.25
			self.touch_sprite.position = self.size / 2
			self.add_child(self.touch_sprite)
			move_action = Action.move_to(-10, -10, 0)
			self.touch_sprite.run_action(move_action)

			self.impactgeneratorlight.impactOccurred()

	def touch_moved(self, touch):
		if len(self.storedTouches) > 0:
			if touch == self.storedTouches[0]:

				previousX = self.current_touch_x
				previousY = self.current_touch_y

				self.current_touch_x, self.current_touch_y = touch.location
				move_action = Action.move_to(self.current_touch_x, self.current_touch_y, 0)
				self.touch_sprite.run_action(move_action)


				if self.current_touch_y - self.touch_start_y > self.deadzone_boundary_y:
				    linearSpeedInTouchIncrements = int((self.current_touch_y - self.touch_start_y + self.deadzone_boundary_y) / self.touch_increment)
				    self.linear_velocity = str(round(float(linearSpeedInTouchIncrements * self.linear_speed_increment), 2))
				    #print(self.linear_velocity)

				elif self.touch_start_y - self.current_touch_y > self.deadzone_boundary_y:
				    linearSpeedInTouchIncrements = int((self.touch_start_y - self.current_touch_y + self.deadzone_boundary_y) / self.touch_increment)
				    self.linear_velocity = str(round(float(linearSpeedInTouchIncrements * -1 * self.linear_speed_increment), 2))
				    #print(self.linear_velocity)

				else:
				    self.linear_velocity = '0.0'



				if self.current_touch_x - self.touch_start_x > self.deadzone_boundary_x:
				    angularSpeedInTouchIncrements = int((self.current_touch_x - self.touch_start_x + self.deadzone_boundary_x) / self.touch_increment)
				    self.angular_velocity = str(round(float(angularSpeedInTouchIncrements * -1 * self.angular_speed_increment), 2))
				    #print(self.angular_velocity)

				elif self.touch_start_x - self.current_touch_x > self.deadzone_boundary_x:
				    angularSpeedInTouchIncrements = int((self.touch_start_x - self.current_touch_x + self.deadzone_boundary_x) / self.touch_increment)
				    self.angular_velocity = str(round(float(angularSpeedInTouchIncrements * self.angular_speed_increment), 2))


				else:
				    self.angular_velocity = '0.0'


				if self.linear_velocity != self.previous_linear_velocity or self.angular_velocity != self.previous_angular_velocity:
				    self.udpClient.sendto(bytes(self.linear_velocity + ' ' + self.angular_velocity, 'utf-8'), (self.servers[0], self.serverPort))


				self.previous_linear_velocity = self.linear_velocity
				self.previous_angular_velocity = self.angular_velocity
				#time.sleep(0.01)

	def touch_ended(self, touch):
		if len(self.storedTouches) > 0:
			if touch == self.storedTouches[0]:
				self.linear_velocity = '0.0'
				self.angular_velocity = '0.0'

				self.current_touch_x = 0
				self.current_touch_y = 0
				self.touch_start_x = 0
				self.touch_start_y = 0

				if self.previous_linear_velocity != '0.0' or self.previous_angular_velocity != '0.0':
				    self.udpClient.sendto(bytes(self.linear_velocity + ' ' + self.angular_velocity, 'utf-8'), (self.servers[0], self.serverPort))

				self.storedTouches.remove(touch)
				self.origin_sprite.remove_from_parent()
				self.touch_sprite.remove_from_parent()
				self.impactgeneratorlight.impactOccurred()

	def draw(self):
		# Line settings
		stroke_weight(1)
		stroke(255, 255, 255)

		if self.touch_start_x != 0 and self.touch_start_y != 0 and self.current_touch_x != 0 and self.current_touch_y != 0:

			# Draw a line between current and origin touch locations
		    line(self.current_touch_x, self.current_touch_y, self.touch_start_x, self.touch_start_y)

			# Display speed
		    text_position_x = self.current_touch_x
		    text_position_y = self.current_touch_y + 80

		    linear_velocity_text = self.linear_velocity + ' m/s'
		    text(linear_velocity_text, font_name='Helvetica', font_size=16.0, x=self.current_touch_x, y=self.current_touch_y + 80, alignment=5)

		    angular_velocity_text = self.angular_velocity + ' rad/s'
		    text(angular_velocity_text, font_name='Helvetica', font_size=16.0, x=self.current_touch_x, y=self.current_touch_y + 60, alignment=5)

if __name__ == '__main__':
	run(MyScene(), show_fps=False, orientation=PORTRAIT)
