#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import Char

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "\nReceived:\n %f\n%f\n%f\n%f\n%f\n%f\n%f\n", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header = std_msgs.msg.Header()
	pose_target.header.frame_id = "base"
	pose_target.header.stamp = rospy.Time.now()
	pose_target.pose = geometry_msgs.msg.Pose()
	pose_target.pose.position = data.position
	pose_target.pose.orientation = data.orientation
	pose_stamped_publisher.publish(pose_target)

def listener():
	rospy.Subscriber("/vr/pose_l", Pose, callback)
	rospy.spin()

def main():
	rospy.init_node('pose_republisher', anonymous=True)

	global pose_stamped_publisher
	pose_stamped_publisher = rospy.Publisher(
		'/vr/pose_l_stamped',
		geometry_msgs.msg.PoseStamped
	)

	print "============ Listening to /vr/pose_l"
	listener()

if __name__ == "__main__":
	main()
