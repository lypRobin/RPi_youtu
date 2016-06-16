#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

VALID_ID = ['20', '00', '00', '08', '04', '00', '00', '00', '07', '5d', '40', '3b', 'd2', '03']

class SerialComm():
	def __init__(self, port = '/dev/ttyS0'):
		self.message = None
		self.port = port
		self.ser = None
		self.node_name = 'Serial'
		self.pub = None

	def initial(self):
		self.ser = serial.Serial(self.port,  \
        						 baudrate = 9600,  \
        						 parity = serial.PARITY_NONE, \
        						 stopbits = serial.STOPBITS_ONE, \
        						 bytesize = serial.EIGHTBITS  \
        						 )
		rospy.init_node(self.node_name, anonymous = True)
		self.pub = rospy.Publisher('message', String, queue_size = 20)

	def measure(self):
		id = []
		line = ''
		while not rospy.is_shutdown():
			data = self.ser.read(14)
			for i in range(len(data)):
				c_hex = str(hex(ord(data[i]))).split('x')
				if len(c_hex[1]) == 1:
					c_hex[1] = '0'+ c_hex[1]
				id.append(c_hex[1])
			
			if len(id) == 14:
				if id == VALID_ID:
					self.pub.publish('card_valid')
				else:
					self.pub.publish('card_invalid')
				self.pub.publish('buzzer_on')

			id = []
				

	def run(self):
		self.measure()

if __name__ == '__main__':
	ser = SerialComm('/dev/ttyS0')
	try:
		ser.initial()
		ser.run()
	except rospy.ROSInterruptException:
		ser.close()