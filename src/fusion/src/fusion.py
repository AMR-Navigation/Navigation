#!/usr/bin/env python

import rospy
from messages.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped

from time import * 
from math import *
from copy import deepcopy


class Fusion:
	def __init__(self):
		rospy.init_node("fusion",anonymous=False)
		rospy.Subscriber("LidarList",UWOList,self.updatelaser)
		rospy.Subscriber("object_detection_results",detection,self.updatedetections)
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)


	def updatelaser(self,data):
		print("Got new laser data:")
		print(data.header)
		print ""

	def updatedetections(self,data):
		print "Got new YOLO data"
		print data.header
		print ""

	def updatepose(self,data):
		print "Got new pose data"
		print data.header
		print ""


if __name__=="__main__":
	F = Fusion()
	print("Initialized.")
	while True:
		#
		sleep(.5)