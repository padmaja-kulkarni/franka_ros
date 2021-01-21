cmake_minimum_required(VERSION 2.8.3)
project(panda_lstm_interface)

find_package(catkin REQUIRED COMPONENTS
    rospy

    gazebo_msgs
    #franka_moveit
    #panda_sim_moveit
    #panda_gazebo
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    gazebo_msgs
    #franka_moveit
    #panda_sim_moveit
    #panda_gazebo
)

install(PROGRAMS
   scripts/move_robot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

