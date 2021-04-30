#!/usr/bin/env python3  

import rospy
from serial import Serial
import struct
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from time import sleep, time
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, Vector3
from std_msgs.msg import UInt32MultiArray, Float32
from nav_msgs.msg import Odometry
import tf
from math import cos, sin, pi 
class SerialControl():
	def __init__(self):
		self.ser = Serial('/dev/ttyACM0', 57600 ,timeout=1.0)
		self.previous_cmd_time = time()
		self.sub_cmd = rospy.Subscriber("cmd_vel_auto", Twist, self.cmd_auto_cb)
		self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
		self.broadcaster = tf.TransformBroadcaster()
		self.previous_cmd_time = time()
		self.previous_odom_time = time()
		self.x = 0
		self.y = 0
		self.theta = 0
		self.nav_state = 'stay' #stay, move, finished
		#self.ir_pub = rospy.Publisher('ir_array', UInt32MultiArray, queue_size=1)
		#self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
		#self.bat_pub = rospy.Publisher('battery_voltage', Float32, queue_size=1)

	def read(self):
		b = str(self.ser.readline())
		b = b.split(';')
		for i in range(0, len(b)-1):
			msg = b[i].split(',')
			if msg[0] == 'tp':
				
			if msg[0] == 'cv':
				self.calc_odom([float(msg[1]), float(msg[2]), float(msg[3])])
		if float(time() - self.previous_cmd_time) > 2.0:
			string = "tv:0.0,0.0,0.0"
			self.ser.write(string.encode())
	def calc_odom(self,velocity):
		delta = time() - self.previous_odom_time
		self.previous_odom_time = time()
		self.theta += delta*velocity[2]
		self.x += delta*(cos(self.theta)*velocity[0] - sin(self.theta)*velocity[1])
		self.y += delta*(sin(self.theta)*velocity[0] + cos(self.theta)*velocity[1])
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta)
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
        	odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(self.x,self.y,0.), Quaternion(*odom_quat))
        	odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(velocity[0],velocity[1],0.),Vector3(0,0,velocity[2]))
		odom.twist.covariance[0] = 0.05
		odom.twist.covariance[7] = 0.05
		odom.twist.covariance[35] = 0.01
		self.broadcaster.sendTransform((self.x,self.y,0.0),odom_quat,rospy.Time.now(),"base_link","odom")
		lidar_quat = tf.transformations.quaternion_from_euler(0,0,pi)
		self.broadcaster.sendTransform((0.0,0.0,0.0),odom_quat,rospy.Time.now(),"base_scan","base_link")
		self.odom_pub.publish(odom)
	def send_cmd(self, data):
		if data.linear.x > 0.22:
			data.linear.x = 0.22
		elif data.linear.x < -0.22:
			data.linear.x = -0.22
		if data.angular.z > 0.57:
			data.angular.z = 0.57
		elif data.angular.z < -0.57:
			data.angular.z = -0.57
		delta = float(time() - self.previous_cmd_time)
		if(delta > 0.2):
			string = "tv:" + str(round(data.linear.x,2)) + "," + str(round(data.linear.y,2)) + "," + str(round(data.angular.z,2))
			print(string)
			self.ser.write(string)
			self.previous_cmd_time = time()
	def cmd_cb(self, data):
		self.send_cmd(data)
	def cmd_auto_cb(self, data):
		self.send_cmd(data)

if __name__ == '__main__':
	rospy.init_node('serial_controller')
	controller = SerialControl()
	while not rospy.is_shutdown():
		try:
			controller.read()
			rospy.sleep(0.05)
		except Exception as e:
			print(e)
