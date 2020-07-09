#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

print("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()


left_arm = moveit_commander.MoveGroupCommander("left_arm")
right_arm = moveit_commander.MoveGroupCommander("right_arm")
right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
left_gripper = moveit_commander.MoveGroupCommander("left_gripper")

print("left:")
print(left_arm.get_current_pose().pose)
print(left_arm.get_current_joint_values())
print("right:")
print(right_arm.get_current_pose().pose)
print(right_arm.get_current_joint_values())
moveit_commander.roscpp_shutdown()

#[-1.2516473449814356, -1.7815064933465452, -0.907947021212931, 0.2207470545428052, 1.470080966734825, 1.4797558598276357, -2.1292445206543933]
#[-1.3570999609519827, -1.9271791737901518, -1.100562806823297, 1.0182158728015551, 1.3728252841839916, 1.4793137079682799, -2.769573412839648]
