#!/usr/bin/env python

import rospy
from messages.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

from time import * 
from math import *
from copy import deepcopy

from laser import *
from yolo_fov import *


class Fusion:
	def __init__(self):
		rospy.init_node("fusion",anonymous=False)
		rospy.Subscriber("LidarList",UWOList,self.updatelaser)
		rospy.Subscriber("object_detection_results",detection,self.updatedetections2)
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)
		self.frame = rospy.Time.now()																		# for synchronization purposes

		# Pose variables
		self.yaw = 0
		self.x = 0
		self.y = 0

		# Laser variables
		self.laserarcs = []
		self.laserobjs = []

		# YOLO variables
		self.detectionarcs = []
		self.detections = []

		# 

	# Arc: (min, max) using the ROS rotation system, e.g. relative to vertical axis, counter-clockwise, and like
	# 			0
	# 	   .5pi    -.5pi
	#			1
	def updatelaser(self,data):
		print("Got new laser data:")
		self.laserarcs = []
		for obj in data.objects:
			self.laserobjs.append(obj)
			self.laserarcs.append( (getarc(obj,self.yaw,self.x,self.y)) )
		#print rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs), '|', self.frame
		print ""

	# Arc: (fov, direction)
	def updatedetections(self,detection):
		pass
		print ("Got new YOLO data")
		self.directionarcs = []
		#for detection in data.detections:
		x = detection.x
		y = detection.y
		width = detection.width
		height = detection.height

		center_x, center_y = box_center(x, y, width, height)
		self.detectionarcs.append( calc_angle(center_x, center_y) )
		self.detections.append(detection)
		print self.detectionarcs
		print self.detections

		print ""

	def updatedetections2(self,detection):
		print("Got new YOLO data")
		print(detection.classification, self.yaw/pi, getarcfrombox(detection.x,detection.width, self.yaw)[0]/pi, getarcfrombox(detection.x,detection.width, self.yaw)[1]/pi)
		self.detections.append(detection)
		self.detectionarcs.append(getarcfrombox(detection.x,detection.width, self.yaw))

	def updatepose(self,data):
		print ("Got new pose data")
		# Set class variables to use in other callbacks
		self.yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		print ""
		self.frame = rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs)								# try to sync with this probably

	def fuse(self):
		pass
		# CURRENT PLAN:
		# sync with detections and laser arcs
		detectionarcs = self.detectionarcs
		laserarcs = self.laserarcs
		laserobjs = self.laserobjs
		# 
		# using previously compiled list as prior,
		# attempt to update the locations of each object
		#for i in range(len(self.record)):
			#for arc in laserarcs:

		# 
		# then take all remaining data and,
		# for every arc in laser arcs:
		# 		attempt to identify it, if impossible then mark as unknown


if __name__=="__main__":
	F = Fusion()
	print("Initialized.")
	while True:
		F.fuse()
		sleep(.5)