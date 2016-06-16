#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time


LIGHT = 13   # GPIO13  pin33
LIGHT_TIME_INTERVAL = 3  # light for 3 sec
class Light():
	def __init__(self, pin):
		self.light = pin
		self.sub = None
		self.node_name = 'Light'
		self.start_time = 0.0

	def listening_callback(self, light_state):
		if light_state.data == 'light_on':
			self.light_on()
			self.start_time = time.time()
		else:
			self.start_time = 0.0
			return


	def initial(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.light, GPIO.OUT)
		self.light_off()

		self.sub = rospy.init_node(self.node_name, anonymous = True)
		rospy.Subscriber('message', String, self.listening_callback)

	def light_on(self):
		GPIO.output(self.light, True)
		print "Light on"

	def light_off(self):
		GPIO.output(self.light, False)
		print "Light off"

	def run(self):
		while not rospy.is_shutdown():
			if self.start_time == 0.0:
				continue
			else:
				t = time.time() - self.start_time
				if t > LIGHT_TIME_INTERVAL:
					self.start_time = 0.0
					self.light_off()
		self.light_off()
		self.start_time = 0.0

if __name__ == '__main__':
	GPIO.setwarnings(False)
	light = Light(LIGHT)
	try:
		light.initial()
		light.run()
	except rospy.ROSInterruptException:
		light.light_off()