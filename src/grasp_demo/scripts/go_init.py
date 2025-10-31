#! /usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf
import time

time.sleep(5)
 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('reset_pose', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")


joint_state_positions = arm_group.get_current_joint_values()
print (str(joint_state_positions))

arm_group.set_joint_value_target([-1.42600646019347, -0.6654795904953197, -2.0600720730880955, -1.9857785438875304, 1.5713970317717658, -1.426795764006920])
arm_group.go(wait=True)

moveit_commander.roscpp_initializer.roscpp_shutdown()