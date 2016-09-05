#!/usr/bin/env python
#
# Copyright (c) 2016, Yanpeng Li <lyp40293@gmail.com>.
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time


BUZZER = 19   # GPIO19  

class Buzzer(object):
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