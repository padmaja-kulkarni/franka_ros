#!/usr/bin/env python


import rospy
import actionlib

from franka_gripper.msg import GraspAction, GraspEpsilon
from franka_gripper.msg import GraspGoal

def grasp_client():
	client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
	print("waiting for server")
	client.wait_for_server()
	eps = GraspEpsilon()
	eps.inner = 0.001
	eps.outer = 0.001
	goal = GraspGoal(width=0.001, epsilon=eps, speed=0.08, force=70.0)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('grasp_client.py')
		result = grasp_client()
		print("successful")
	except rospy.ROSInterruptException:
		print("Not successful")
	
	
