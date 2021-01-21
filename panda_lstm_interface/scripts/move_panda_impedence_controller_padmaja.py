#! /usr/bin/env python

import copy
import rospy
import threading
#import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
#from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import tf


class PubGoal:
    
    def __init__(self):
        
        
        self.listener = tf.TransformListener()
        self.base_link = 'panda_EE'
        self.ee_pose =  'panda_link0' ##TODO check this
        self.goal_offset = 0.03
	self.axis_offset = 0.03
        self.cart_pose_array = np.ones((1,3)) * self.axis_offset
        self.goal_publisher = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size =10)
    
    def getTFdata(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.ee_pose, self.base_link,rospy.Time(0) )
            curr_pose = np.array(sum([trans, rot], [])).reshape(1,-1)
            return curr_pose 
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return []
            pass       
        
    def goal_pose_in_cart(self, goal_pos):
        curr_pose = []
        print('getting tf data')
        while len(curr_pose) == 0:
            curr_pose = self.getTFdata()
        print('Got tf data',goal_pos, curr_pose )
        goal_pose_t = goal_pos - curr_pose[0, :3]
        
        print( "actual goal pose: goal_pose_t", goal_pose_t)
        goal_pose_t_offset = copy.deepcopy(goal_pose_t)
        
        for i in range(goal_pose_t.shape[1]):
            if goal_pose_t[0][i] <= self.axis_offset and goal_pose_t[0][i] >= -self.axis_offset:
                pass
            else:
                goal_pose_t_offset[0][i] = np.copysign(self.axis_offset, goal_pose_t[0][i])
                      
        print( "Later goal pose: goal_pose_t", goal_pose_t_offset)
        
        if True: #np.linalg.norm(goal_pose_t, 1) > self.goal_offset:
            goal = PoseStamped()
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = ''
            goal.pose.position.x = curr_pose[0][0] + goal_pose_t_offset[0][0] 
            goal.pose.position.y = curr_pose[0][1] + goal_pose_t_offset[0][1] 
            goal.pose.position.z = curr_pose[0][2] + goal_pose_t_offset[0][2]
            
            goal.pose.orientation.x = 1.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
	    goal.pose.orientation.w = 0.0
	    print("Publishing goal")
            self.goal_publisher.publish(goal)
            
                
if __name__ == "__main__":
    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("publish_goal")
    
    rate = rospy.Rate(10)
    
    pub_goal = PubGoal()
    
    goal = np.array([[0.61, 0.0, 0.20]])
    #goal = np.array([[0.71, 0.0, 0.10]])
        
    while not rospy.is_shutdown():
        pub_goal.goal_pose_in_cart(goal)
        rate.sleep()
        
    rospy.spin()
    
    # if not using franka_ros_interface, you have to subscribe to the right topics
    # to obtain the current end-effector state and robot jacobian for computing 
    # commands
    

    
