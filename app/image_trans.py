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

import os
import struct
import socket 
import hashlib
import rospy
from std_msgs.msg import String
import time

HOST = '192.168.1.60'
# HOST = 'localhost'
PORT = 19888
BUFFER_SIZE = 1024
FILE_NAME = 'image.jpg'
IMAGE_FILE = '/home/pi/.ros/' + FILE_NAME
FILE_SIZE = os.path.getsize(IMAGE_FILE)


class TransitionClient(object):
	def __init__(self):
		self.node_name = 'transfer'
		self.sub = None
		self.pub = None
		self.server_address = (HOST,PORT)

	def listening_callback(self, data):
		if data.data == 'trans':
			self.send_file()

	def initial(self):
		rospy.init_node(self.node_name, anonymous = True)
		self.sub = rospy.Subscriber('message', String, self.listening_callback)
		self.pub = rospy.Publisher('message', String, queue_size = 30)

	def check_file(self, file):
		if os.path.exists(file):
			return True
		else:
			return False

	def send_file(self):
		if not self.check_file(IMAGE_FILE):
			print 'Image file does not exists.'
			return
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		print 'Calculating MD5 value......'
		with open(IMAGE_FILE, 'rb') as f:
			md5_code = hashlib.md5()
			md5_code.update(f.read())

		with open(IMAGE_FILE, 'rb') as f:
			# try:
			print 'Connect to ' + HOST + ':' + str(PORT)
			self.sock.connect(self.server_address)
			send_info = FILE_NAME + '=' + str(FILE_SIZE) + '=' + md5_code.hexdigest()
			self.sock.send(send_info)
			time.sleep(0.001)
			lock = self.sock.recv(BUFFER_SIZE)
			if lock == 'lock':
				self.pub.publish('image_lock')

			send_size = 0
			print 'Sending data......'
			while(send_size < FILE_SIZE):
				if(FILE_SIZE - send_size < BUFFER_SIZE):
					file_data = f.read(FILE_SIZE - send_size)
					send_size = FILE_SIZE
				else:
					file_data = f.read(BUFFER_SIZE)
					send_size += BUFFER_SIZE
				self.sock.send(file_data)
				print 'Send size is: ' + str(send_size)
			# except:
			# 	print 'Transfer image file failed.'
			# 	return

		person_id = self.sock.recv(BUFFER_SIZE)
		if person_id == 'null':
			self.pub.publish('person_id_null')
		else:
			person = person_id.split('=')[0]
			self.pub.publish('validperson_liyanpeng')

		unlock = self.sock.recv(BUFFER_SIZE)
		if unlock == 'unlock':
			self.pub.publish('image_unlock')

		self.sock.close()

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	tc = TransitionClient()
	tc.initial()
	tc.run()