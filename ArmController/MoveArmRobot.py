#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
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

jointvalues = left_arm.get_current_joint_values()
jointValues = [-1.2516473449814356, -1.7815064933465452, -0.907947021212931, 0.2207470545428052, 1.470080966734825, 1.4797558598276357, -2.1292445206543933]
left_arm.go(jointValues, wait=True)

jointValues = [-1.3570999609519827, -1.9271791737901518, -1.100562806823297, 1.0182158728015551, 1.3728252841839916, 1.4793137079682799, -2.769573412839648]
left_arm.go(jointValues, wait=True)

jointValues = [-1.2516473449814356, -1.7815064933465452, -0.907947021212931, 0.2207470545428052, 1.470080966734825, 1.4797558598276357, -2.1292445206543933]
left_arm.go(jointValues, wait=True)

jointValues = [-1.3570999609519827, -1.9271791737901518, -1.100562806823297, 1.0182158728015551, 1.3728252841839916, 1.4793137079682799, -2.769573412839648]
left_arm.go(jointValues, wait=True)

rospy.sleep(6)

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0.510722
p.pose.position.y = -0.445874
p.pose.position.z = 0.153950
scene.add_box("box", p, (0.5,0.4,0.3))



moveit_commander.roscpp_shutdown()
