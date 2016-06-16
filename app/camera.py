#!/usr/bin/env python

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


class Camera():
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

	def node_callback(self, data):
		if data.data == 'distance_ok':
			if self.enable_show:
				self.start_time = time.time()
				self.enable_read = True
		elif data.data == 'enable_show_video':
			self.enable_show = True
		elif data.data == 'card_valid' or data.data == 'card_invalid':
			self.enable_show = False


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
		while not rospy.is_shutdown():
			cv2.waitKey(30)
			print 'Camera node still working...'
			if self.enable_read:
				t = time.time() - self.start_time
				print 'Enable read image.'
				print 'Start time: ' + str(self.start_time) + ', Current time: ' + str(time.time())
				if t < SHOW_VIDEO_DELAY and t > 0:
					if self.cap.isOpened():
						pass
					else:
						try:
							self.cap.open(self.dev)
							if not self.cap.isOpened():
								print 'Open camera failed.'
								# return
							else:
								# self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1024)
								# self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 720)

								self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
								self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
						except:
							print 'open camera failed, return!'
							return

					ret, img = self.cap.read()
					print 'Image read from camera!'
					img = cv2.flip(img, 1)
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

					cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
					cv2.setWindowProperty(self.window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)	#opencv3.0
					cv2.setMouseCallback(self.window, self.mouse_click_callback)
					# cv2.setWindowProperty(self.window, cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)  # opencv 2.4
					cv2.imshow(self.window, img_show)
					
					screen = wnck.screen_get_default()
					screen.force_update()
					for s in screen.get_windows():
						if s.get_name().find(self.window) != -1:
							s.activate(0)
				else:
					time.sleep(0.01)
					if self.cap.isOpened():
						self.cap.release()
						self.start_time = 0.0
						self.enable_read = False
						cv2.destroyWindow(self.window)
						print 'time up stop show image'
			else:
				if self.cap.isOpened():
					self.cap.release()
					self.start_time = 0.0
					cv2.destroyWindow(self.window)
					print 'disable read stop show image'


		try:
			if self.cap.isOpened():
				self.cap.release()
				self.start_time = 0.0
				cv2.destroyWindow(self.window)
				print 'stop show image'

		except:
			return

if __name__ == '__main__':
	cam = Camera(CAMERA_DEV)
	if not cam.initial():
		print "fail initial"
	else:
		cam.run()
