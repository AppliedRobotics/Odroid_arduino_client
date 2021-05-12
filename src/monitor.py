#!/usr/bin/env python  
import rospy
import struct
from time import sleep, time
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
class Monitor():
	def __init__(self):
		self.sub_1 = rospy.Subscriber("scan", LaserScan, self.laser_cb)
		self.sub_2 = rospy.Subscriber("odom", Odometry, self.odom_cb)
		self.sub_3 = rospy.Subscriber("serial_status", Bool, self.serial_cb)
		self.sub_4 = rospy.Subscriber("nav_status", String, self.nav_cb)
		self.status = {'laser': [time(), False],
		'odom': [time(), False],
		'serial': [time(), False]}
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
		self.nav_state = {'state': 'none', 
		'x_target': 0.0, 
		'y_target:': 0.0, 
		'theta_target': 0.0,
		'x_real': 0.0,
		'y_real': 0.0,
		'theta_real': 0.0}
	def laser_cb(self, data):	
		self.status['laser'][0] = time()
	def odom_cb(self, data):
		self.status['odom'][0] = time()
	def serial_cb(self, data):
		self.status['serial'][0] = time()
	def nav_cb(self,data):
		b = data.data.split(';')
		#print(b)
		for s in b:
			s_ = s.split(':')
			for key in self.nav_state:
				if key == s_[0]:
					if isinstance(self.nav_state[key], str):
						self.nav_state[key] = s_[1]
					else:
						self.nav_state[key] = float(s_[1])
	def check_time(self, key):
		t = self.status[key][0]
		if time() - t > 0.5:
			self.status[key][1] = False
		else:
			self.status[key][1] = True
	def timer_callback(self, timer):
		for key in self.status:
			self.check_time(key)
	def get_nav_state(self):
               return self.nav_state
        
        def get_status(self):
        	return self.status

if __name__ == '__main__':
	rospy.init_node('status_checker')
	mon = Monitor()
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except Exception as e:
			print(e)
