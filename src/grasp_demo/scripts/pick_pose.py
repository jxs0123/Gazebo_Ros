#! /usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf

 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('reset_pose', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")


joint_state_positions = arm_group.get_current_joint_values()
print (str(joint_state_positions))


arm_group.set_joint_value_target([-0.2366192138065646, -1.6647092498036438, 1.6630617121944766, -3.0694460917878734, -2.9044654189365504, -3.0608461287977935])
# arm_group.go(wait=True)

p = arm_group.get_current_pose()
print(p)

# p.pose.orientation.x = -0.707
# p.pose.orientation.y = 0.707
# p.pose.orientation.z = 0.0
# p.pose.orientation.w = 0

# arm_group.set_pose_target(p.pose)
# arm_group.go()


moveit_commander.roscpp_initializer.roscpp_shutdown()