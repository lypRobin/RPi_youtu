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

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
import time
import wnck
import random
from threading import Timer

CAMERA_DEV = 0  # /dev/video0
SHOW_VIDEO_DELAY = 3.0  #  delay 3 seconds when stop catching sonar distance callback
AVERAGE_Y_POINTS = 100  #  Randomly choose points to calculate average Y value


class Camera(object):
	def __init__(self, dev):
		self.dev = dev
		self.node_name = 'Camera'
		self.light_pub = None
		self.pub = None
		self.window = 'camera_video'
		self.cap = cv2.VideoCapture()
		self.enable_read = False
		self.enable_show = True
		self.start_time = 0.0
		self.image_lock = False

	def node_callback(self, data):
		if data.data == 'distance_ok':
			if self.enable_show:
				self.start_time = time.time()
				print '[Camera] camera catch distance ok'
				self.enable_read = True
		elif data.data == 'enable_show_video':
			self.enable_show = True
		elif data.data == 'card_valid' or data.data == 'card_invalid':
			self.enable_show = False
		elif data.data == 'image_lock':
			self.image_lock = True
		elif data.data == 'image_unlock' or data.data == 'person_id_null':
			self.image_lock = False
		elif data.data == 'validperson_liyanpeng':
			self.enable_show = False
			self.enable_read = False


	def mouse_click_callback(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.enable_show = False
			self.enable_read = False
			self.pub.publish('show_option')


	def initial(self):
		rospy.init_node(self.node_name, anonymous = True)
		self.pub = rospy.Publisher('message', String, queue_size = 20)
		self.sub = rospy.Subscriber('message', String, self.node_callback)
		
		return True

	def run(self):
		is_win_create = False
		frame_cnt = 0

		while not rospy.is_shutdown():
			cv2.waitKey(30)
			print '[Camera] Camera node still working...'
			if self.enable_read:
				t = time.time() - self.start_time
				print '[Camera] Enable read image.'
				if t < SHOW_VIDEO_DELAY and t > 0:
					if self.cap.isOpened():
						pass
					else:
						try:
							self.cap.open(self.dev)
							if not self.cap.isOpened():
								print '[Camera] Open camera failed.'
								return
							else:
								self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
								self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
						except:
							print '[Camera] open camera failed, return!'
							return

					ret, img = self.cap.read()
					print 'Image read from camera!'
					img = cv2.flip(img, 1)


					frame_cnt += 1
					if frame_cnt % 30 == 0:
						# store_img = img[:, 242:540]
						cv2.imwrite('./image.jpg', img[:, 242:782])
						# cv2.imwrite('./image.jpg', img)
						time.sleep(0.01)
						self.pub.publish('trans')
					
					h, w = img.shape[:2]
					res = cv2.resize(img, None, fx = 0.44, fy = 0.44, interpolation = cv2.INTER_CUBIC)  # scaled 720p image to a 320 height image
					img_show = res[0:320, 107:347]			# split the 320 height image to 240 width
					yuv = cv2.cvtColor(img_show, cv2.COLOR_RGB2YUV)    # convert to the yuv format image to calculate the Y channel
					h, w = yuv.shape[:2]
					
					# Randomly choose 100 points to calculate average Y value determining the brightness of the image
					# Maybe not scientific, but the MCU loads too much to calculate average Y value of the whole image
					y_avr = 0.0					
					for i in range(100):   
						y_avr += yuv[random.randint(0,h-1), random.randint(0,w-1), 0]

					y_avr = y_avr / 100
					print "Y average is: " + str(y_avr)
					if y_avr < 10.0:
						self.pub.publish('light_on')

					if not is_win_create:
						cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
						cv2.setWindowProperty(self.window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
						cv2.setMouseCallback(self.window, self.mouse_click_callback)
						is_win_create = True

					cv2.imshow(self.window, img_show)

					screen = wnck.screen_get_default()
					screen.force_update()
					for s in screen.get_windows():
						if s.get_name().find(self.window) != -1:
							if not s.is_active():
								s.activate(0)
				else:
					time.sleep(0.01)
					frame_cnt = 0
					if self.cap.isOpened():
						self.cap.release()
						self.start_time = 0.0
						self.enable_read = False
						cv2.destroyWindow(self.window)
						is_win_create = False
					print '[Camera] time up stop show image'
			else:
				frame_cnt = 0
				if self.cap.isOpened():
					self.cap.release()
					self.start_time = 0.0
					cv2.destroyWindow(self.window)
					is_win_create = False
				print '[Camera] disable read stop show image'


		try:
			frame_cnt = 0
			if self.cap.isOpened():
				self.cap.release()
				self.start_time = 0.0
				cv2.destroyWindow(self.window)
				is_win_create = False
				print '[Camera] stop show image'

		except:
			return

if __name__ == '__main__':
	cam = Camera(CAMERA_DEV)
	if not cam.initial():
		print "[Camera] fail initial"
	else:
		cam.run()
