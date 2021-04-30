#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def cbcmd(data):
	pub_vel.publish(data)
if __name__ == '__main__':
	rospy.init_node('cmd_proxy')
	sub_cmd = rospy.Subscriber('auto_cmd_vel', Twist, cbcmd, queue_size=1)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			velocity = Twist()
			velocity.angular.z = 0
			velocity.linear.x = 0
			pub_vel.publish(velocity)
			break
