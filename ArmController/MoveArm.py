#!/usr/bin/env python

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Float64
from std_msgs.msg import String
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Bartender',
                    anonymous=True)

    left_gripper = rospy.Publisher('/yumi/gripper_effort_controller_l/command', Float64, queue_size=1)
    right_gripper = rospy.Publisher('/yumi/gripper_effort_controller_r/command', Float64, queue_size=1)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    right_arm = moveit_commander.MoveGroupCommander("right_arm")

    def let_it_go_let_it_go_oh_wont_you_just_let_it_go(data):
        while left_gripper.get_num_connections() < 1: pass
        left_gripper.publish(open)

        time.sleep(1)

        pub = rospy.Publisher('done_waiting_waiter', String, queue_size=10)
        while pub.get_num_connections() < 1: pass
        pub.publish("Go for it")

        p =[1.6212253948417779, -0.8188849586602083, -1.0347596190756603, 0.12322793731718384, -3.6657287205509164, 0.6565071773074829, -1.4228637374595126]
        left_arm.go(p, wait=True)
        left_arm.stop()
        moveit_commander.roscpp_shutdown()
        s = String()
        s.data = "done"
        rospy.signal_shutdown(s)
        return

    open = Float64()
    open.data = 1
    closed = Float64()
    closed.data = -1

    while right_gripper.get_num_connections() < 1: pass
    right_gripper.publish(open)

    while left_gripper.get_num_connections() < 1: pass
    left_gripper.publish(open)

    right_arm.set_goal_tolerance(0.001)
    p = [0.011577985417905445, -2.079045110906201, -0.6970964112272942, 0.26967512450401454, -1.0087831859202225, 0.0012938773129436143, -1]
    right_arm.go(p, wait=True)
    right_arm.stop()

    left_arm.set_goal_tolerance(0.001)
    p = [-1.4066738734955422, -2.097086102922941, 0.7068506691303336, 0.2969133174791141, 6.109471263116006e-06, -1.0126910385821475e-06, -1]
    left_arm.go(p, wait=True)
    left_arm.stop()

    scene.remove_world_object("box")

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.510722
    p.pose.position.y = -0.445874
    p.pose.position.z = 0.153950
    scene.add_box("box", p, (0.5,0.4,0.3))

    #Move into position above can
    p = [0.5721191367180776, -1.6293475323130049, -0.8026125618483446, 0.5375496431117011, -2.609835298746324, -0.11250495578863173, -0.680767863842207]
    right_arm.go(p, wait=True)
    right_arm.stop()

    #move down over can
    p = [0.6283651575631488, -1.6041347556722667, -1.0900084690260536, 0.4241253362916213, -2.7504517097293677, -0.10589884276431238, -0.7141011742203114]
    right_arm.go(p, wait=True)
    right_arm.stop()

    #grab can
    while right_gripper.get_num_connections() < 1: pass
    right_gripper.publish(closed)

    time.sleep(1)

    #move up
    p = [0.6967317874188526, -1.7110294112625497, -0.9102712083416566, 0.4398034114623286, -2.738249462661983, -0.18137966833259966, -0.7363196788018085]
    right_arm.go(p, wait=True)
    right_arm.stop()

    #position left arm to grab can
    p = [-2.15309519932552, -2.0098483629402346, 1.0540885106961468, -0.04600461412114498, -0.7142583235872051, 0.9811875300620105, -0.7324386804294516]
    left_arm.go(p, wait=True)
    left_arm.stop()

    p = [1.3219613395781185, 0.31502071681509713, -1.6844877686135176, 0.7250357817413544, 3.2523612093072254, 1.6440864949093719, -1.8879289614634898]
    right_arm.go(p, wait=True)
    right_arm.stop()


    p = [-2.1143588319022157, -2.0501278589623504, 1.0490919428750907, 0.17731691625245904, -0.7668052950768018, 0.8308768833445459, -0.6149275755649803]
    left_arm.go(p, wait=True)
    left_arm.stop()

    #pass can between hands
    while left_gripper.get_num_connections() < 1: pass
    left_gripper.publish(closed)

    time.sleep(0.5)

    while right_gripper.get_num_connections() < 1: pass
    right_gripper.publish(open)


    time.sleep(1)

    p = [2.2870473540811336, 0.4297016634643298, -2.4093448782432585, 0.9916275372197942, 3.454623795176232, 1.65989605513884, -2.07841849928932]
    right_arm.go(p, wait=True)
    right_arm.stop()



    p = [1.1472085384397603, -1.5921049052131475, -0.638105182392775, 0.28284725523000187, -0.45787949162645347, -0.19316049985293304, 0.007893944207087777]
    right_arm.go(p, wait=True)
    right_arm.stop()

    pub = rospy.Publisher('done_waiting_waiter', String, queue_size=10)
    while pub.get_num_connections() < 1: pass
    pub.publish("YumiTime")
    rospy.Subscriber("waiting_waiter", String, let_it_go_let_it_go_oh_wont_you_just_let_it_go)

    #place can on turtlebot
    p =[1.981309832588117, -0.7472656442507071, -1.3710094069231111, 0.15447230188559935, -3.628508063638627, 0.9477534390116684, -1.5262279485423225]
    left_arm.go(p, wait=True)
    left_arm.stop()
    rospy.spin()



    # p = Pose()
    # p.position.z = 0.40
    # right_arm.go(p, wait=True)





    # grasps = []
    #
    # g = Grasp()
    # g.id = "canGrab"
    # p = PoseStamped()
    # p.header.frame_id = robot.get_planning_frame()
    # p.pose.position.x = 0.31451061922
    # p.pose.position.y = -0.42295176596
    # p.pose.position.z = 0.391565528678
    # p.pose.orientation.x = 0.556517924705
    # p.pose.orientation.y = 0.646703259532
    # p.pose.orientation.z = 0.393280953283
    # p.pose.orientation.w = 0.342626305729
    # #right_arm.go(p, wait=True)
    # g.grasp_pose = p
    # #
    # g.pre_grasp_approach.direction.header.frame_id = robot.get_planning_frame()
    # g.pre_grasp_approach.direction.vector.z = -1.0
    # g.pre_grasp_approach.min_distance = 0.005
    # g.pre_grasp_approach.desired_distance = 0.05
    # #g.pre_grasp_posture.header.frame_id = "gripper_r_base1"
    # #g.pre_grasp_posture.joint_names = ["gripper_r_finger_l", "gripper_r_finger_r"]
    # #
    # #pos = JointTrajectoryPoint()
    # #pos.positions.append(0.0)
    #
    # #g.pre_grasp_posture.points.append(pos)
    #
    # #g.grasp_posture.header.frame_id = "gripper_r_base"
    # joint_names = ["gripper_r_finger_l", "gripper_r_finger_r"]
    #
    # #pos = JointTrajectoryPoint()
    # #pos.positions.append(0.2)
    # #pos.effort.append(0.0)
    #
    # #g.grasp_posture.points.append(pos)
    #
    # g.post_grasp_retreat.direction.header.frame_id = robot.get_planning_frame()
    # g.post_grasp_retreat.direction.vector.z = 1.0
    # g.post_grasp_retreat.desired_distance = 0.05
    # g.post_grasp_retreat.min_distance = 0.005
    #
    # g.max_contact_force = 0
    #
    # g.allowed_touch_objects = ["can"]
    #
    # #grasps.append(g)
    #
    # right_arm.set_planner_id("RRTkConfigDefault")
    #
    # right_arm.set_support_surface_name("box")
    #
    # result = right_arm.pick("can", g)
    #
    #
    # result = -1
    # n_attempts = 0
    #
    # # repeat until will succeed
    # while result == -1:
    #     result = right_arm.pick("can", grasps)
    #     print(result)
    #     n_attempts += 1
    #     print "Attempts: ", n_attempts
    #     rospy.sleep(0.2)
    #
    #
    # rospy.spin()


    # RightWaypoints = list()
    #
    # print("============ Generating plan_left ============")
    # RightWaypoints.append(right_arm.get_current_pose().pose)
    # print(right_arm.get_current_pose().pose)
    #
    # pose_target = right_arm.get_current_pose().pose
    # pose_target.position.x = 0.228
    # pose_target.position.y = -0.392
    # pose_target.position.z = 0.293
    #
    # pose_target.orientation.x = 0.535
    # pose_target.orientation.y = -0.553
    # pose_target.orientation.z = 0.462
    # pose_target.orientation.w = -0.441
    # RightWaypoints.append(copy.deepcopy(pose_target))
    #
    # #pose_target.position.z = 0.410
    # #RightWaypoints.append(copy.deepcopy(pose_target))
    # #group_variable_values[0] = 0.3
    # #group_left.set_joint_value_target(group_variable_values)
    # #plan_right = group_left.plan()
    # fraction = 0
    # count = 10
    # while fraction != 1.0 and count > 0:
    #     (plan, fraction) = right_arm.compute_cartesian_path(RightWaypoints, 0.01, 0.0)
    #     count-=1

    # right_arm.set_planner_id("RRTConnectkConfigDefault")
    #
    # pose_target = right_arm.get_current_pose().pose
    # pose_target.position.x = 0.312851
    # pose_target.position.y = -0.379462
    # pose_target.position.z = 0.315853
    #
    # pose_target.orientation.x = -0.486384
    # pose_target.orientation.y = 0.515611
    # pose_target.orientation.z = -0.576499
    # pose_target.orientation.w = 0.406478
    #
    # #right_arm.goal_tolerance = 0.001
    #
    # right_arm.set_pose_target(pose_target)
    # plan = right_arm.plan()
    # print(plan)
    # print("============ Waiting while RVIZ displays plan_left... ============")
    # rospy.sleep(5)
    # #group_right.move(plan)
    # right_arm.execute(plan)
    print("============ Visualizing plan_left ============")
    #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = robot.get_current_state()
    #display_trajectory.trajectory.append(plan)
    #display_trajectory_publisher.publish(display_trajectory)

    #print("============ Waiting while plan_left is visualized (again)... ============")
    #rospy.sleep(10)



if __name__ == "__main__":
    main()
