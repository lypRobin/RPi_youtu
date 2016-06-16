#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time


BUZZER = 19   # GPIO19  

class Buzzer():
	def __init__(self, pin):
		self.buzzer = pin
		self.sub = None
		self.node_name = 'Buzzer'

	def node_callback(self, buzzer_state):
		if buzzer_state.data == 'buzzer_on':
			self.buzzer_on()
			time.sleep(0.2)
			self.buzzer_off()
		else:
			return

	def initial(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.buzzer, GPIO.OUT)
		GPIO.output(self.buzzer, False)

		rospy.init_node(self.node_name, anonymous = True)
		self.sub = rospy.Subscriber('message', String, self.node_callback)

	def buzzer_on(self):
		GPIO.output(self.buzzer, True)
		print "Buzzer on"

	def buzzer_off(self):
		GPIO.output(self.buzzer, False)
		print "Buzzer off"

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	GPIO.setwarnings(False)
	buzzer = Buzzer(BUZZER)
	try:
		buzzer.initial()
		buzzer.run()
	except rospy.ROSInterruptException:
		buzzer.buzzer_off()