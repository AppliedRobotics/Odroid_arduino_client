#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from rns_msgs.msg import MoveToActionGoal, MoveToActionFeedback, MoveToAction, MoveToActionResult, MoveToGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from std_msgs.msg import String
from math import sin, cos
class NavControl():
    def __init__(self):
        #rospy.init_node('nav_controller')   
        print('start control') 
        self.sub_cmd = rospy.Subscriber('/move_state', String, self.cbfeedback, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_action/goal', MoveToActionGoal, queue_size=1)
        self.client_move_base = actionlib.SimpleActionClient('move_action',MoveToAction)
        self.client_move_base.wait_for_server()
        self.feedback = 'empty'
        #print("succeded connect to move base")
    def cbfeedback(self, data):
        self.feedback = data.data

    def get_feedback(self):
        return self.feedback

    def sendgoal(self, x = 0, y = 0, alpha = 0):
        goal = MoveToGoal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = rospy.Time.now()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.position.z = 0

        goal.goal.pose.orientation.x = 0
        goal.goal.pose.orientation.y = 0
        goal.goal.pose.orientation.z = sin(alpha / 2)
        goal.goal.pose.orientation.w = cos(alpha / 2)
        self.client_move_base.send_goal(goal, feedback_cb=self.cbfeedback)
        # wait = self.client_move_base.wait_for_result()
        # print(wait)
if __name__ == '__main__':
    con = NavControl()
    con.sendgoal(0.4, 0.4, 0.3)
    rospy.spin()

