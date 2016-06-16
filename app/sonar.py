#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time


TRIGGER = 5    # GPIO5  pin29
ECHO = 12     # GPIO12  pin32

class DistanceMeasurement():
	def __init__(self, trigger, echo):
		self.trigger = trigger
		self.echo = echo
		self.distance = 0.0
		self.pub = None
		self.node_name = 'Sonar'

	def initial(self):
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.trigger, GPIO.OUT)
		GPIO.setup(self.echo, GPIO.IN)

		self.pub = rospy.Publisher('message', String, queue_size = 20)
		rospy.init_node(self.node_name, anonymous=True)


	def measure(self):
		# GPIO.output(self.trigger, False)
		# time.sleep(0.0001)

		GPIO.output(self.trigger, True)
		time.sleep(0.00002)
		print 'pre measure give high'
		GPIO.output(self.trigger, False)
		print 'pre measure give low'

		timeout_start = time.time()
		while GPIO.input(self.echo) == 0:
			if time.time() - timeout_start > 1.0:
				return False
			pass
		start = time.time()
		print 'get start time'

		timeout_start = time.time()
		while GPIO.input(self.echo) == 1:
			if time.time() - timeout_start > 1.0:
				return False
			pass
		stop = time.time()
		print 'get stop time'

		elapsed = stop - start
		self.distance = (elapsed * 34000) / 2
		print "Distance: %.1f" % self.distance

		return True

	def run(self):
		while not rospy.is_shutdown():
			if self.measure():
				if self.distance > 20.0 and self.distance < 50.0:  # distance range in [20, 50] cm
					self.pub.publish("distance_ok")
					print 'Publish distance ok'
			time.sleep(0.2)


if __name__ == '__main__':
	try:
		GPIO.setwarnings(False)
		sonar = DistanceMeasurement(TRIGGER, ECHO)
		sonar.initial()
		sonar.run()
	except rospy.ROSInterruptException:
		pass