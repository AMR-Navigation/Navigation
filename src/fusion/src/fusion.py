#!/usr/bin/env python

import rospy
from messages.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

from time import * 
from math import *
from copy import deepcopy

from laser import *


class Fusion:
	def __init__(self):
		rospy.init_node("fusion",anonymous=False)
		rospy.Subscriber("LidarList",UWOList,self.updatelaser)
		rospy.Subscriber("object_detection_results",detection,self.updatedetections)
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)
		self.frame = rospy.Time.now()																		# for synchronization purposes

		# Pose variables
		self.yaw = 0
		self.x = 0
		self.y = 0

		# Laser variables
		self.laserarcs = []


	def updatelaser(self,data):
		print("Got new laser data:")
		self.laserarcs = []
		for obj in data.objects:
			self.laserarcs.append( getarc(obj,self.yaw,self.x,self.y) )
		#print rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs), '|', self.frame
		print ""

	def updatedetections(self,data):
		print "Got new YOLO data"
		print data.header
		print ""

	def updatepose(self,data):
		print "Got new pose data"
		self.yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		print ""
		self.frame = rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs)


if __name__=="__main__":
	F = Fusion()
	print("Initialized.")
	while True:
		#
		sleep(.5)