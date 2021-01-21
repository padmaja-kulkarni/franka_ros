#! /usr/bin/env python

import copy
import rospy
import threading
import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import tf


import pickle
import matplotlib.pyplot as plt
from os.path import isfile, join
from os import listdir
import sklearn
from sklearn import svm
from sklearn import preprocessing
from sklearn.preprocessing import Normalizer
from sklearn.model_sele#! /usr/bin/env python

import copy
import rospy
import threading
import quaternion
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
import tf


import pickle
import matplotlib.pyplot as plt
from os.path import isfile, join
from os import listdir
import sklearn
from sklearn import svm
from sklearn import preprocessing
from sklearn.preprocessing import Normalizer
from sklearn.model_selection import train_test_split
from sklearn.svm import SVR
import tensorflow as tff
from tf.transformations import quaternion_matrix, quaternion_inverse, quaternion_multiply, euler_from_quaternion
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from sklearn.neural_network import MLPRegressor
import numpy as np
from sklearn.metrics import mean_squared_error
from sklearn.datasets import make_friedman1
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.ensemble import RandomForestRegressor
from sklearn.datasets import make_regression
from sklearn.decomposition import KernelPCA

from tensorflow import keras
from tensorflow.keras import layers

DATA_LEN = 30

class GetForcePos:
    
    def __init__(self):
        
        rospy.Subscriber('/franka_state_controller/F_ext',
                                                    WrenchStamped, self.getForceCB, queue_size=1, tcp_nodelay=True)
        
        self.listener = tf.TransformListener()
                
        self.goal_pos_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, self.goalPoseCB, queue_size=1)
        
        event_sub = rospy.Subscriber('/move_robot_with_impedence_control_sim/e_event', String, self.eventCB, queue_size=1)
        
        # create joint command message and fix its type to joint torque mode
        rospy.loginfo("Subscribing to robot force and data topics")
        
        self.data_len = copy.deepcopy(DATA_LEN)
        self.force_list = np.empty((0, 6))
        self.curr_poses_list = np.empty((0, 7))
        self.equilibirum_pose_list = np.empty((0, 7))
        self.base_link = 'panda_link0'
        self.ee_pose = 'panda_link7' ##TODO check this
            
        
        
    def getForceCB(self, msg):
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        if len(self.force_list) == self.data_len:
            self.force_list = np.delete(self.force_list, 1, 0)
        self.force_list = np.vstack((self.force_list, force))      
        
    def getForceList(self):
        return self.force_list 
    
    def getTFdata(self):
        try:
            (trans,rot) = self.listener.lookupTransform(self.ee_pose, self.base_link,rospy.Time(0) )
            curr_pose = np.array(sum([trans, rot], [])).reshape(1,-1)
            if len(self.curr_poses_list) == self.data_len:
                self.curr_poses_list = np.delete(self.curr_poses_list, 1, 0)
            self.curr_poses_list = np.vstack((self.curr_poses_list, curr_pose)) 
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass       
        
    def getCurrentPoseList(self):
        self.getTFdata()
        return self.curr_poses_list 

    def goalPoseCB(self, msg):
        equilibirum_pose = np.array([msg.pose.position.x,msg.pose.position.y, msg.pose.position.z,
                                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])     
        if len(self.equilibirum_pose_list) == self.data_len:
            self.equilibirum_pose_list = np.delete(self.equilibirum_pose_list, 1, 0)
        self.equilibirum_pose_list = np.vstack((self.equilibirum_pose_list, equilibirum_pose))
        
        
    def eventCB(self, msg):
        self.event = msg.data    
        
        
class LSTMModelTest:
    
    def __init__(self):
        self.model = keras.models.load_model('/home/padmaja/ros/src/panda_padmaja/DataAnalysis/lstm_model_30_len.h5')
        
    def predict(self, data):
         return self.model.predict(data)
         


if __name__ == "__main__":
    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("get_force_pose")
    
    rate = rospy.Rate(30)
    
    get_data = GetForcePos()
    
    lstm_model = LSTMModelTest()
    
    while not rospy.is_shutdown():
        forces = get_data.getForceList()
        if forces.shape[0] == DATA_LEN:
            print("prediction", lstm_model.predict(np.expand_dims(forces, axis=0)))
        rate.sleep()
        
    rospy.spin()ction import train_test_split
from sklearn.svm import SVR
import tensorflow as tff
from tf.transformations import quaternion_matrix, quaternion_inverse, quaternion_multiply, euler_from_quaternion
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from sklearn.neural_network import MLPRegressor
import numpy as np
from sklearn.metrics import mean_squared_error
from sklearn.datasets import make_friedman1
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.ensemble import RandomForestRegressor
from sklearn.datasets import make_regression
from sklearn.decomposition import KernelPCA

from tensorflow import keras
from tensorflow.keras import layers

DATA_LEN = 30

MAX_EFFORT = 20

class CloseFingers:
    
    def __init__(self):
        self.pubFinger1 = rospy.Publisher("/mmrobot/panda_finger1_effort_controller/command",Float64, queue_size=10)
        self.pubFinger2 = rospy.Publisher("/mmrobot/panda_finger2_effort_controller/command",Float64, queue_size=10)
        
        
    def move_gripper_effort(self, effort):
        self.pubFinger1.publish(effort)
        self.pubFinger2.publish(effort)

 

    def close_gripper(self):
            #self.move_gripper([0.0, 0.0])
        self.move_gripper_effort(Float64(data=-1.0))

 


    def open_gripper(self):
            #self.move_gripper([0.04, 0.04])
        self.move_gripper_effort(Float64(data=1.0))
         

if __name__ == "__main__":
    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("get_force_pose")
    
    rate = rospy.Rate(30)
   
    while not rospy.is_shutdown():
        rate.sleep()
        
    rospy.spin()
    
    # if not using franka_ros_interface, you have to subscribe to the right topics
    # to obtain the current end-effector state and robot jacobian for computing 
    # commands
    

    