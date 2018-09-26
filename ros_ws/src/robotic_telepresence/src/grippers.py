#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
from std_msgs.msg import Char
import baxter_interface
import baxter_external_devices
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION

def callback_l(data):
	if data.data == 103: # 'g'
		print "Grabbing!"
		left.close()
	elif data.data == 117: # 'u'
		print "Ungrabbing!"
		left.open()

def callback_r(data):
	if data.data == 103: # 'g'
		print "Grabbing!"
		right.close()
	elif data.data == 117: # 'u'
		print "Ungrabbing!"
		right.open()

def listener():
	rospy.Subscriber("/vr/grab_l", Char, callback_l)
	rospy.Subscriber("/vr/grab_r", Char, callback_r)
	rospy.spin()

def main():
	rospy.init_node('vr_left_grabber', anonymous=True)
	print "============ Initialising gripper parameters"
	global left
	global right
	left = baxter_interface.Gripper('left', CHECK_VERSION)
	right = baxter_interface.Gripper('right', CHECK_VERSION)
	left.calibrate()
	right.calibrate()
	print "============ Listening to /vr/grab_l"
	print "============ Listening to /vr/grab_r"
	listener()

if __name__ == "__main__":
	main()

