#!/usr/bin/env python  
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
from nav_msgs.msg import OccupancyGrid
from math import cos, sin, pi 
from nav_controller import NavControl
from rnsslam.msg import ScanPose
class SerialControl():
	def __init__(self):
		connected = False
		while connected == False:
			try:
				self.ser = Serial('/dev/ttyACM0', 9600 ,timeout = 0.05)
				connected = True
			except Exception as e:
				print(e)
				rospy.sleep(0.5)
		print("connected")
		self.nav = NavControl()
		self.previous_cmd_time = time()
		self.sub_cmd_1 = rospy.Subscriber("auto_cmd_vel", Twist, self.cmd_auto_cb)
		self.sub_cmd_2 = rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
		self.sub_map = rospy.Subscriber("map", OccupancyGrid, self.map_cb)
		self.pose_cb = rospy.Subscriber("scanpose", ScanPose, self.scanpose_cb)
		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
		self.broadcaster = tf.TransformBroadcaster()
		self.previous_cmd_time = time()
		self.previous_odom_time = time()
		self.x_real = 0
		self.y_real = 0
		self.theta_real = 0
		self.x = 0
		self.y = 0
		self.theta = 0
		self.vx = 0
		self.vy = 0
		self.wz = 0
		self.nav_state = 'not' #stay, move, finished
		self.x_target = 0.0
		self.y_target = 0.0
		self.theta_target = 0.0
		self.new_point_flag = False
		self.start_msg = False
		#self.ir_pub = rospy.Publisher('ir_array', UInt32MultiArray, queue_size=1)
		#self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
		#self.bat_pub = rospy.Publisher('battery_voltage', Float32, queue_size=1)
	def map_cb(self, data):
		if self.start_msg == False:
			self.nav_state = "empty"
			self.start_msg = True
	def scanpose_cb(self,data):
		self.x_real = data.pose.pose.position.x
		self.y_real = data.pose.pose.position.y
		self.theta_real = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w))
	def read(self):
		b = str(self.ser.readline())
		#print(b)
		b = b.split(';')
		for i in range(0, len(b)):
			msg = b[i].split(',')
			if msg[0] == 'tp':
				#a = 1
				self.analize_target([float(msg[1]), float(msg[2]), float(msg[3])])
			if msg[0] == 'cv':
				self.calc_odom([float(msg[1]), float(msg[2]), float(msg[3])])
		if float(time() - self.previous_cmd_time) > 2.0:
			self.vx = 0
			self.vy = 0 
			self.wz = 0
		#print(self.nav_state)
		string_ = "tv:" + str(round(self.vx,2)) + "," + str(round(self.vy,2)) + "," + str(round(self.wz,2))+";s:"+self.nav_state+";rp:"+str(self.x_real)+','+str(self.y_real)+','+str(self.theta)
		#string_ = "tv:" + str(round(self.vx,2)) + "," + str(round(self.vy,2)) + "," + str(round(self.wz,2))
		#print(string_)
		self.ser.write(string_)
	def analize_target(self, point):
		#print(self.x_target, self.y_target, self.theta_target, self.nav_state)
		#print(point[0], point[1], point[2])
		if (point[0] != self.x_target or point[1] != self.y_target or point[2] != self.theta_target) and self.new_point_flag == False:
			self.new_point_flag = True
		if self.start_msg == True:
			if self.nav_state != 'moving' and self.new_point_flag == True:
				self.x_target = point[0]
				self.y_target = point[1]
				self.theta_target = point[2]
				print(self.x_target, self.y_target, self.theta_target)
				self.nav.sendgoal(self.x_target, self.y_target, self.theta_target)
				#rospy.sleep(0.1)
				self.nav_state = self.nav.get_feedback()
			else:
				print(self.x_target, self.y_target, self.theta_target)
				self.nav_state = self.nav.get_feedback()
				print(self.nav_state)
				if self.nav_state == 'finished':
					self.new_point_flag = False
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
					print(" ")
				#string = "s:" + self.nav_state
				#self.ser.write(string)
				#string = "rp:"+str(self.x_real)+','+str(self.y_real)+','+str(self.theta)
				#self.ser.write(string)
				#print(self.nav_state)
	def calc_odom(self,velocity):
		delta = time() - self.previous_odom_time
		self.previous_odom_time = time()
		self.theta -= delta*velocity[2]
		self.x += delta*(cos(self.theta)*velocity[0] - sin(self.theta)*velocity[1])
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
		lidar_quat = tf.transformations.quaternion_from_euler(0,0,0)
		self.broadcaster.sendTransform((0.0,0.0,0.0),lidar_quat,rospy.Time.now(),"base_scan","base_link")
		self.odom_pub.publish(odom)
	def send_cmd(self, data):
		if data.linear.x > 0.12:
			data.linear.x = 0.12
		elif data.linear.x < -0.12:
			data.linear.x = -0.12
		if data.angular.z > 0.57:
			data.angular.z = 0.57
		elif data.angular.z < -0.57:
			data.angular.z = -0.57
		delta = float(time() - self.previous_cmd_time)
		#if(delta > 0.2):
		#print(delta)
		self.vx = data.linear.x
		self.vy = data.linear.y
		self.wz = data.angular.z 
		#string = "tv:" + str(round(data.linear.x,2)) + "," + str(round(data.linear.y,2)) + "," + str(round(data.angular.z,2))+"\n"
		#self.ser.write(string)
		self.previous_cmd_time = time()
	def cmd_cb(self, data):
		self.send_cmd(data)
	def cmd_auto_cb(self, data):
		if data.linear.x != 0.0 or data.angular.z != 0.0:
			self.send_cmd(data)

if __name__ == '__main__':
	rospy.init_node('serial_controller')
	controller = SerialControl()
	while not rospy.is_shutdown():
		try:
			controller.read()
			rospy.sleep(0.1)
		except Exception as e:
			print(e)
