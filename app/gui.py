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

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from PyQt4 import uic
import rospy
from std_msgs.msg import String
import sys, os
import shutil, urllib
import time

OPTION_ACCESS_PASSWORD = 0
OPTION_SET_ADMIN_PASSWORD = 1
OPTION_SET_HOST_IP = 2
OPTION_SET_LOCAL_IP = 3
OPTION_SET_ACCESS_PASSWORD = 4

ACCESS_PASSWORD_LENGTH = 6
ADMIN_PASSWORD_LENGTH = 6

TIMER_INTERVAL = 2
VERSION_URL = 'https://raw.githubusercontent.com/lypRobin/RPi_youtu/master/VERSION'
YOUTU_APP_DIR = '/home/pi/Documents/RPi_youtu/app/src/youtu/'
APP_CONFIG_FILE = YOUTU_APP_DIR+'config/gui.config'


class Updater(object):
	def __init__(self):
		self.cur_version = ''
		self.new_version = ''

	def get_version(self):
		return self.new_version

	def check_network(self):
		try:
			ret = os.system('ping www.baidu.com -c 2')
			if ret:
				return False
			else:
				return True
		except:
			return False

	def check_version(self):
		with open(APP_CONFIG_FILE,'r') as f:
			for line in f.readlines():
				if 'version=' in line:
					try:
						self.cur_version = line.split('=')[1]
					except:
						self.cur_version = ''

		print '[Updater] Get current version'
		try:
			versions = urllib.urlopen(VERSION_URL).readlines()
			print '[Updater] Reading new version'

			for line in versions:
				if 'version=' in line:
					self.new_version = line.split('=')[1]
			if self.new_version != '' and self.cur_version != '':
				if float(self.new_version) > float(self.cur_version):
					return 1
			else:
				print '[Updater] Version file invalid' 
				return -2
		except:
			print '[Updater] check version failed.'
			return -1

		print '[Updater] same version'
		return 0

	def update(self):
		try:
			if os.path.exists(YOUTU_APP_DIR+'/temp'):
				shutil.rmtree(YOUTU_APP_DIR+'/temp')
			os.system('git clone https://github.com/lypRobin/RPi_youtu.git ' + YOUTU_APP_DIR+'/temp')
			os.system('cp -r ' + YOUTU_APP_DIR+'/temp/app/* '+ YOUTU_APP_DIR+'/scripts')
		except:
			print '[Updater] Update failed.'
			return False

		try:
			if os.path.exists(YOUTU_APP_DIR+'/temp'):
				shutil.rmtree(YOUTU_APP_DIR+'/temp')
		except:
			print '[Updater] Update failed.'
			return False

		lines = []
		with open(APP_CONFIG_FILE,'r') as f:
			i = 0
			lines = f.readlines()
			for line in lines:
				if 'version=' in line:
					lines[i] = 'version=' + self.new_version
				i += 1

		with open(APP_CONFIG_FILE, 'w') as f:
			f.writelines(lines)

		return True


class MyWindow(QMainWindow):
	def __init__(self):
		super(MyWindow, self).__init__()
		uic.loadUi(YOUTU_APP_DIR + 'scripts/gui.ui', self)

		# ROS 
		self.node_name = 'GUI'
		self.show_video_pub = None
		self.gui_sub = None
		self.card_sub = None


		# Private variables
		self.access_password = ''  
		self.admin_password = ''

		self.addr = ['','','','']
		self.addr_input = [self.addr0_input, self.addr1_input, self.addr2_input,self.addr3_input]
		self.addr_idx = 0

		self.new_pw = ['','']
		self.pw_idx = 0
		self.timer = QTimer()
		self.update_timer = QTimer()
		self.operation_state = ''
		self.options = ''   # determine whick page to enter after get admin authority
		self.card_state = ''    # IC card state to determine which page to change after success or failed.
		self.updater = Updater()

		QObject.connect(self.timer, SIGNAL("timeout()"), self.timing)
		QObject.connect(self.update_timer, SIGNAL("timeout()"), self.update_timing)

		self.addr0_input.installEventFilter(self)
		self.addr1_input.installEventFilter(self)
		self.addr2_input.installEventFilter(self)
		self.addr3_input.installEventFilter(self)

		self.pw_input0.installEventFilter(self)
		self.pw_input1.installEventFilter(self)

		self.mainPage.installEventFilter(self)

		# main page 		

		# option page
		self.back_btn.clicked.connect(self.goto_main_page)
		self.accesspw_btn.clicked.connect(self.goto_accesspw_page)
		self.set_hostip_btn.clicked.connect(lambda: self.goto_adminpw_page('host_ip'))
		self.set_localip_btn.clicked.connect(lambda: self.goto_adminpw_page('local_ip'))
		self.set_accesspw_btn.clicked.connect(lambda: self.goto_adminpw_page('access_pw'))
		self.set_adminpw_btn.clicked.connect(lambda: self.goto_adminpw_page('admin_pw'))
		self.update_btn.clicked.connect(lambda: self.goto_adminpw_page('update'))

		# Access Password Page
		self.acc_pw_btn0.clicked.connect(lambda: self.access_pw_input('0'))
		self.acc_pw_btn1.clicked.connect(lambda: self.access_pw_input('1'))
		self.acc_pw_btn2.clicked.connect(lambda: self.access_pw_input('2'))
		self.acc_pw_btn3.clicked.connect(lambda: self.access_pw_input('3'))
		self.acc_pw_btn4.clicked.connect(lambda: self.access_pw_input('4'))
		self.acc_pw_btn5.clicked.connect(lambda: self.access_pw_input('5'))
		self.acc_pw_btn6.clicked.connect(lambda: self.access_pw_input('6'))
		self.acc_pw_btn7.clicked.connect(lambda: self.access_pw_input('7'))
		self.acc_pw_btn8.clicked.connect(lambda: self.access_pw_input('8'))
		self.acc_pw_btn9.clicked.connect(lambda: self.access_pw_input('9'))
		self.acc_pw_delete_btn.clicked.connect(self.access_pw_delete)
		self.acc_pw_reset_btn.clicked.connect(self.access_pw_reset)
		self.acc_pw_ok_btn.clicked.connect(self.access_pw_ok)
		self.acc_pw_cancel_btn.clicked.connect(self.access_pw_cancel)

		# Admin Password Page
		self.admin_pw_btn0.clicked.connect(lambda: self.admin_pw_input('0'))
		self.admin_pw_btn1.clicked.connect(lambda: self.admin_pw_input('1'))
		self.admin_pw_btn2.clicked.connect(lambda: self.admin_pw_input('2'))
		self.admin_pw_btn3.clicked.connect(lambda: self.admin_pw_input('3'))
		self.admin_pw_btn4.clicked.connect(lambda: self.admin_pw_input('4'))
		self.admin_pw_btn5.clicked.connect(lambda: self.admin_pw_input('5'))
		self.admin_pw_btn6.clicked.connect(lambda: self.admin_pw_input('6'))
		self.admin_pw_btn7.clicked.connect(lambda: self.admin_pw_input('7'))
		self.admin_pw_btn8.clicked.connect(lambda: self.admin_pw_input('8'))
		self.admin_pw_btn9.clicked.connect(lambda: self.admin_pw_input('9'))
		self.admin_pw_delete_btn.clicked.connect(self.admin_pw_delete)
		self.admin_pw_reset_btn.clicked.connect(self.admin_pw_reset)
		self.admin_pw_ok_btn.clicked.connect(self.admin_pw_ok)
		self.admin_pw_cancel_btn.clicked.connect(self.admin_pw_cancel)

		# Set IP Address Page
		self.set_ip_btn0.clicked.connect(lambda: self.set_ip_input('0'))
		self.set_ip_btn1.clicked.connect(lambda: self.set_ip_input('1'))
		self.set_ip_btn2.clicked.connect(lambda: self.set_ip_input('2'))
		self.set_ip_btn3.clicked.connect(lambda: self.set_ip_input('3'))
		self.set_ip_btn4.clicked.connect(lambda: self.set_ip_input('4'))
		self.set_ip_btn5.clicked.connect(lambda: self.set_ip_input('5'))
		self.set_ip_btn6.clicked.connect(lambda: self.set_ip_input('6'))
		self.set_ip_btn7.clicked.connect(lambda: self.set_ip_input('7'))
		self.set_ip_btn8.clicked.connect(lambda: self.set_ip_input('8'))
		self.set_ip_btn9.clicked.connect(lambda: self.set_ip_input('9'))
		self.set_ip_delete_btn.clicked.connect(self.set_ip_delete)
		self.set_ip_reset_btn.clicked.connect(self.set_ip_reset)
		self.set_ip_ok_btn.clicked.connect(self.set_ip_ok)
		self.set_ip_cancel_btn.clicked.connect(self.set_ip_cancel)

		# Set Password Page
		self.set_pw_btn0.clicked.connect(lambda: self.set_pw_input('0'))
		self.set_pw_btn1.clicked.connect(lambda: self.set_pw_input('1'))
		self.set_pw_btn2.clicked.connect(lambda: self.set_pw_input('2'))
		self.set_pw_btn3.clicked.connect(lambda: self.set_pw_input('3'))
		self.set_pw_btn4.clicked.connect(lambda: self.set_pw_input('4'))
		self.set_pw_btn5.clicked.connect(lambda: self.set_pw_input('5'))
		self.set_pw_btn6.clicked.connect(lambda: self.set_pw_input('6'))
		self.set_pw_btn7.clicked.connect(lambda: self.set_pw_input('7'))
		self.set_pw_btn8.clicked.connect(lambda: self.set_pw_input('8'))
		self.set_pw_btn9.clicked.connect(lambda: self.set_pw_input('9'))
		self.set_pw_delete_btn.clicked.connect(self.set_pw_delete)
		self.set_pw_reset_btn.clicked.connect(self.set_pw_reset)
		self.set_pw_ok_btn.clicked.connect(self.set_pw_ok)
		self.set_pw_cancel_btn.clicked.connect(self.set_pw_cancel)

	# Initial Section
	def eventFilter(self, object, event):
		if object is self.addr0_input:
			if event.type() == QEvent.FocusIn:
				self.addr_idx = 0
		elif object is self.addr1_input:
			if event.type() == QEvent.FocusIn:
				self.addr_idx = 1
		elif object is self.addr2_input:
			if event.type() == QEvent.FocusIn:
				self.addr_idx = 2
		elif object is self.addr3_input:
			if event.type() == QEvent.FocusIn:
				self.addr_idx = 3
		elif object is self.pw_input0:
			if event.type() == QEvent.FocusIn:
				self.pw_idx = 0
		elif object is self.pw_input1:
			if event.type() == QEvent.FocusIn:
				self.pw_idx = 1
		elif object is self.mainPage:
			if event.type() == QEvent.MouseButtonPress:
				self.goto_option_page()

		return QWidget.eventFilter(self, object, event)

	def init_main_page(self):
		self.mainPage.setStyleSheet(".QWidget {background-image: url(/home/pi/Documents/RPi_youtu/app/src/youtu/scripts/images/main.png)}")

		pass

	def init_option_page(self):
		pass

	def init_success_page(self):
		self.successPage.setStyleSheet(".QWidget {background-image: url(/home/pi/Documents/RPi_youtu/app/src/youtu/scripts/images/welcome.png)}")

	def init_fail_page(self):
		self.failPage.setStyleSheet(".QWidget {background-image: url(/home/pi/Documents/RPi_youtu/app/src/youtu/scripts/images/failed.png)}")

	def init_accesspw_page(self):
		pass

	def init_adminpw_page(self):
		pass

	def init_setip_page(self):
		pass

	def init_setpw_page(self):
		pass

	def initial(self):
		self.Pages = {'mainPage':0, \
					  'optionPage':1, \
					  'updatePage':2, \
					  'successPage':3, \
					  'failPage':4, \
					  'accessPasswordPage':5, \
					  'adminPasswordPage':6, \
					  'setIPPage':7, \
					  'setPasswordPage':8 }

		rospy.init_node(self.node_name, anonymous = True)
		self.pub = rospy.Publisher('message', String, queue_size = 20)
		self.sub = rospy.Subscriber('message', String, self.show_callback)
		# self.card_sub = rospy.Subscriber('access', String, self.card_access_callback)

		self.init_main_page()
		self.init_option_page()
		self.init_success_page()
		self.init_fail_page()
		self.init_accesspw_page()
		self.init_adminpw_page()
		self.init_setpw_page()
		self.init_setip_page()

	def click_main_page(self):
		self.goto_option_page()

	def show_callback(self, data):
		if data.data == 'show':
			self.goto_main_page()
		elif data.data == 'nshow':
			pass
		elif data.data == 'show_option':
			self.goto_option_page()
		elif data.data == 'card_valid':
			self.goto_success_page()
			self.card_state = 'card_varify'
		elif data.data == 'validperson_liyanpeng':
			self.goto_success_page()
		elif data.data == 'card_invalid':
			self.goto_failed_page()
			self.card_state == 'card_varify'


	def timing(self):
		self.timer.stop()
		if self.operation_state == 'failed':
			if self.card_state == 'card_varify':
				self.goto_main_page()
				self.card_state = ''
			else:
				self.goto_option_page()

		elif self.operation_state == 'success':
			self.goto_main_page()
		self.operation_state = ''

	def update_timing(self):
		self.update_timer.stop()
		need_update = True
		self.version_info.setText('Checking network...')
		if not self.updater.check_network():
			self.version_info.setText('Please check network')
			self.operation_state = 'failed'
			self.timer.start(2000)
			need_update = False

		self.version_info.setText('Checking version...')
		ret = self.updater.check_version()
		if ret == 0:
			self.version_info.setText('The Newest Version')
			self.version_num.setText('Version: ' + self.updater.get_version())
			self.operation_state = 'success'
			self.timer.start(2000)
			need_update = False

		elif ret < 0:
			self.version_info.setText('Checking Version Failed.')
			self.operation_state = 'failed'
			self.timer.start(2000)
			need_update = False
		else:
			self.version_info.setText('Updating...')

		if need_update:
			if self.updater.update():
				self.version_info.setText('Update Success!')
				self.version_num.setText('Version: ' + self.updater.get_version())
				self.operation_state = 'success'
				time.sleep(2)
				self.version_info.setText('Success! Please Reboot')
			else:
				self.version_info.setText('Update Failed.')
				self.operation_state = 'failed'
				self.timer.start(2000)
			

	# Page convertion
	def goto_option_page(self):
		self.stackedWidget.setCurrentIndex(self.Pages['optionPage'])

	def goto_main_page(self):
		self.pub.publish('enable_show_video')
		time.sleep(0.1)
		self.stackedWidget.setCurrentIndex(self.Pages['mainPage'])

	def goto_update_page(self):
		self.stackedWidget.setCurrentIndex(self.Pages['updatePage'])
		self.update_timer.start(100)
		

	def goto_accesspw_page(self):
		self.acc_pw_input.setAlignment(Qt.AlignCenter)
		self.stackedWidget.setCurrentIndex(self.Pages['accessPasswordPage'])

	def goto_failed_page(self):
		self.stackedWidget.setCurrentIndex(self.Pages['failPage'])
		self.operation_state = 'failed'
		self.timer.start(2000)

	def goto_success_page(self):
		self.stackedWidget.setCurrentIndex(self.Pages['successPage'])
		self.operation_state = 'success'
		self.timer.start(2000)

	def goto_adminpw_page(self, option):
		if 'ip' in option:
			self.options = option
		elif 'pw' in option:
			self.options = option
		elif 'update' in option:
			self.options = option

		self.stackedWidget.setCurrentIndex(self.Pages['adminPasswordPage'])

	def goto_setip_page(self):
		
		with open(APP_CONFIG_FILE,'r') as f:
			if self.options == 'host_ip':
				for line in f.readlines():
					if 'host_ip' in line:
						ip = line.split('=')[1].strip()
						self.addr = ip.split('.')
				self.set_ip_label.setText('set host ip')
			elif self.options == 'local_ip':
				for line in f.readlines():
					if 'local_ip' in line:
						ip = line.split('=')[1].strip()
						self.addr = ip.split('.')
				self.set_ip_label.setText('set local ip')

		self.ip_input_update()

		self.addr_idx = 0
		self.addr0_input.setTextCursor(QTextCursor())
		self.stackedWidget.setCurrentIndex(self.Pages['setIPPage']) 

	def goto_setpw_page(self):
		
		if self.options == 'access_pw':
			self.set_pw_label.setText('Set Access Password')
		elif self.options == 'admin_pw':
			self.set_pw_label.setText('Set Admin Password')
			
		self.pw_idx = 0		
		self.pw_input0.setTextCursor(QTextCursor())
		self.stackedWidget.setCurrentIndex(self.Pages['setPasswordPage']) 

	# Access Password Page
	def access_pw_input(self, input):
		if len(self.access_password) < 6:
			self.access_password += input

		self.acc_pw_input.setText('*'*len(self.access_password))

	def access_pw_delete(self):
		if len(self.access_password) > 0:
			self.access_password = self.access_password[:-1]

		self.acc_pw_input.setText('*'*len(self.access_password))

	def access_pw_reset(self):
		self.access_password = ''
		self.acc_pw_input.setText('')

	def access_pw_ok(self):
		pw = ''
		with open(APP_CONFIG_FILE,'r') as f:
			for line in f.readlines():
				if 'access_password' in line:
					pw = line.split('=')[1].strip()

		if pw == '':
			pass

		if self.access_password == pw and len(pw) == ACCESS_PASSWORD_LENGTH:
			self.access_pw_reset()
			self.goto_success_page()
		else:
			self.access_pw_reset()
			self.goto_failed_page()

	def access_pw_cancel(self):
		self.access_password = ''
		self.acc_pw_input.setText('')
		self.goto_option_page()


	# Admin Password Page
	def admin_pw_input(self, input):
		if len(self.admin_password) < ADMIN_PASSWORD_LENGTH:
			self.admin_password += input

		self.adm_pw_input.setText('*'*len(self.admin_password))

	def admin_pw_delete(self):
		if len(self.admin_password) > 0:
			self.admin_password = self.admin_password[:-1]

		self.adm_pw_input.setText('*'*len(self.admin_password))

	def admin_pw_reset(self):
		self.admin_password = ''
		self.adm_pw_input.setText('')

	def admin_pw_ok(self):
		pw = ''
		with open(APP_CONFIG_FILE,'r') as f:
			for line in f.readlines():
				if 'admin_password' in line:
					pw = line.split('=')[1].strip()

		if pw == '':
			pass

		if self.admin_password == pw and len(pw) == ADMIN_PASSWORD_LENGTH:
			self.admin_pw_reset()
			if self.options == 'host_ip' or self.options == 'local_ip':
				self.goto_setip_page()
			elif self.options == 'access_pw' or self.options == 'admin_pw':
				self.goto_setpw_page()
			elif self.options == 'update':
				self.goto_update_page()
		else:
			self.admin_pw_reset()
			self.goto_failed_page()

	def admin_pw_cancel(self):
		self.admin_password = ''
		self.adm_pw_input.setText('')
		self.goto_option_page()



	# Set IP Address Page
	def ip_input_update(self):
		for i in range(4):
			self.addr_input[i].setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}") 	
			self.addr_input[i].setText(self.addr[i])
			if (len(self.addr[i]) >= 2 and self.addr[i][0] == '0') or \
				(len(self.addr[i]) > 0 and int(self.addr[i]) > 255):
				self.addr_input[i].setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")


	def set_ip_input(self, input):
		if len(self.addr[self.addr_idx]) < 3:
			self.addr[self.addr_idx] += input

		self.ip_input_update()

	def set_ip_delete(self):
		if len(self.addr[self.addr_idx]) > 0:
			self.addr[self.addr_idx] = self.addr[self.addr_idx][:-1]

		self.ip_input_update()
	def set_ip_reset(self):
		self.addr = ['','','','']
		self.ip_input_update()
		
	def set_ip_ok(self):
		for i in range(4):	
			self.addr_input[i].setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
			if (len(self.addr[i]) >= 2 and self.addr[i][0] == '0') or \
				(len(self.addr[i]) > 0 and int(self.addr[i]) > 255) or \
				len(self.addr[i]) == 0:
				self.addr_input[i].setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")
				return

		lines = []
		with open(APP_CONFIG_FILE, 'r') as f:
			i = 0
			lines = f.readlines()
			for line in lines:
				if self.options == 'host_ip':
					if 'host_ip' in line:
						lines[i] = 'host_ip='+self.addr[0]+'.'+self.addr[1]+'.'+self.addr[2]+'.'+self.addr[3]+'\n'
				elif self.options == 'local_ip':
					if 'local_ip' in line:
						lines[i] = 'local_ip='+self.addr[0]+'.'+self.addr[1]+'.'+self.addr[2]+'.'+self.addr[3]+'\n'
				i += 1

		with open(APP_CONFIG_FILE, 'w') as f:
			f.writelines(lines)

		self.addr = ['','','','']
		self.ip_input_update()
		self.options = ''
		self.goto_option_page()

	def set_ip_cancel(self):
		self.addr = ['','','','']
		self.ip_input_update()
		self.options = ''
		self.goto_option_page()

	# Set New Password Page
	def set_pw_input(self, input):
		self.pw_input0.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		self.pw_input1.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		if self.options == 'access_pw':
			if len(self.new_pw[self.pw_idx]) < ACCESS_PASSWORD_LENGTH:
				self.new_pw[self.pw_idx] += input

		elif self.options == 'admin_pw':
			if len(self.new_pw[self.pw_idx]) < ADMIN_PASSWORD_LENGTH:
				self.new_pw[self.pw_idx] += input

		self.pw_input0.setText('*'*len(self.new_pw[0]))
		self.pw_input1.setText('*'*len(self.new_pw[1]))

	def set_pw_delete(self):
		self.pw_input0.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		self.pw_input1.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		if self.new_pw[self.pw_idx] > 0:
			self.new_pw[self.pw_idx] = self.new_pw[self.pw_idx][:-1]
		self.pw_input0.setText('*'*len(self.new_pw[0]))
		self.pw_input1.setText('*'*len(self.new_pw[1]))

	def set_pw_reset(self):
		self.pw_input0.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		self.pw_input1.setStyleSheet(".QTextEdit {background-color: rgb(255, 255, 255)}")
		self.new_pw = ['','']
		self.pw_input0.setText('')
		self.pw_input1.setText('')

	def set_pw_ok(self):
		if self.options == 'access_pw':
			if self.new_pw[0] == self.new_pw[1] and len(self.new_pw[0]) == ACCESS_PASSWORD_LENGTH:
				lines = []
				with open(APP_CONFIG_FILE, 'r') as f:
					lines = f.readlines()
					i = 0
					for line in lines:
						if 'access_password' in line:
							lines[i] = 'access_password='+self.new_pw[0]+'\n'
						i += 1

				with open(APP_CONFIG_FILE,'w') as f:
					f.writelines(lines)
			else:
				self.pw_input0.setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")
				self.pw_input1.setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")
				return
		if self.options == 'admin_pw':
			if self.new_pw[0] == self.new_pw[1] and len(self.new_pw[0]) == ADMIN_PASSWORD_LENGTH:
				lines = []
				with open(APP_CONFIG_FILE, 'r') as f:
					lines = f.readlines()
					i = 0
					for line in lines:
						if 'admin_password' in line:
							lines[i] = 'admin_password='+self.new_pw[0]+'\n'
						i += 1

				with open(APP_CONFIG_FILE,'w') as f:
					f.writelines(lines)
			else:
				self.pw_input0.setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")
				self.pw_input1.setStyleSheet(".QTextEdit {background-color: rgb(255, 0, 0)}")	
				return
		
		self.new_pw = ['','']
		self.pw_input0.setText('')
		self.pw_input1.setText('')
		self.pw_idx = 0
		self.options = ''
		self.goto_option_page()		

	def set_pw_cancel(self):
		self.new_pw = ['','']
		self.pw_input0.setText('')
		self.pw_input1.setText('')
		self.pw_idx = 0
		self.options = ''
		self.goto_option_page()


	# Final Execute
	def	execute(self):
		self.initial()
		self.stackedWidget.setStyleSheet(".QStackedWidget {background-image: url(/home/pi/Documents/RPi_youtu/app/src/youtu/scripts/images/main.png)}")	
		self.stackedWidget.setCurrentIndex(0)
		
		# self.setWindowFlags(Qt.WindowStaysOnBottomHint)
		self.showFullScreen()
		self.stackedWidget.show()

if __name__ == '__main__':
	app = QApplication(sys.argv)
	app.setOverrideCursor(QCursor(Qt.BlankCursor))

	window = MyWindow()
	window.execute()
	sys.exit(app.exec_())