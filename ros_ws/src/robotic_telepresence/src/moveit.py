#!/usr/bin/env python
# cd /vol/vssp/baxter/ros/moveit_ws/
# source devel/setup.sh
# roslaunch baxter_moveit_config baxter_grippers.launch
# then this one

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Char

def callback_l(data):
	rospy.loginfo(rospy.get_caller_id() + "\nReceived:\n %f\n%f\n%f\n%f\n%f\n%f\n%f\n", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	pose_target = geometry_msgs.msg.Pose()
	#pose_target.orientation.w = 1.0
	#pose_target.position.x = 0.7
	#pose_target.position.y = -0.05
	#pose_target.position.z = 1.1
	#group.set_pose_target(pose_target)
	pose_target.position = data.position
	pose_target.orientation = data.orientation
	group_l.set_pose_target(pose_target)

	plan1 = group_l.plan()
	#print "============ Waiting while RVIZ displays plan1..."
	#rospy.sleep(5)

	print "============ Visualizing plan1"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher_l.publish(display_trajectory)

	print "============ Waiting while plan1 is visualized (again)..."
	#rospy.sleep(1)

	#ik_status_l_publisher.publish('c')
	#ik_status_l_publisher.publish('f')

	# Uncomment below line when working with a real robot
	group_l.go(wait=True)

def callback_r(data):
	rospy.loginfo(rospy.get_caller_id() + "\nReceived:\n %f\n%f\n%f\n%f\n%f\n%f\n%f\n", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	pose_target = geometry_msgs.msg.Pose()
	#pose_target.orientation.w = 1.0
	#pose_target.position.x = 0.7
	#pose_target.position.y = -0.05
	#pose_target.position.z = 1.1
	#group.set_pose_target(pose_target)
	pose_target.position = data.position
	pose_target.orientation = data.orientation
	group_r.set_pose_target(pose_target)

	plan1 = group_r.plan()
	#print "============ Waiting while RVIZ displays plan1..."
	#rospy.sleep(5)

	print "============ Visualizing plan1"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher_r.publish(display_trajectory)

	print "============ Waiting while plan1 is visualized (again)..."
	#rospy.sleep(1)

	#ik_status_l_publisher.publish('c')
	#ik_status_l_publisher.publish('f')

	# Uncomment below line when working with a real robot
	group_r.go(wait=True)


def listener():
	rospy.Subscriber("/vr/pose_l", Pose, callback_l)
	rospy.Subscriber("/vr/pose_r", Pose, callback_r)
	rospy.spin()

def main():
	print "============ Starting setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('arms_interface_vr',
					anonymous=True)

	global robot
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	global group_l
	group_l = moveit_commander.MoveGroupCommander("left_arm")
	global group_r
	group_r = moveit_commander.MoveGroupCommander("right_arm")
	global display_trajectory_publisher_l
	display_trajectory_publisher_l = rospy.Publisher(
		'/left_arm/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory
	)
	global display_trajectory_publisher_r
	display_trajectory_publisher_r = rospy.Publisher(
		'/right_arm/display_planned_path',
		moveit_msgs.msg.DisplayTrajectory
	)
	global ik_status_l_publisher
	ik_status_l_publisher = rospy.Publisher(
		'/vr/ik_status_l',
		std_msgs.msg.Char
	)
	global ik_status_r_publisher
	ik_status_r_publisher = rospy.Publisher(
		'/vr/ik_status_r',
		std_msgs.msg.Char
	)

	print "============ Waiting for RViz..."
	rospy.sleep(3)
	print "============ Starting"
	print "============ Reference frame: %s" % group_l.get_planning_frame()
	print "============ Reference frame: %s" % group_r.get_planning_frame()
	print "============ Reference frame: %s" % group_l.get_end_effector_link()
	print "============ Reference frame: %s" % group_r.get_end_effector_link()
	print "============ Robot Groups:"
	print robot.get_group_names()
	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"

	# print "============ Generating (example) plan 1"
	print "============ Listening to /vr/pose_l"
	print "============ Listening to /vr/pose_r"
	listener()

if __name__ == "__main__":
	main()
