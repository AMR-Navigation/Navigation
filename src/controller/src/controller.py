#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

import map

import time
from math import *
from copy import deepcopy
import numpy
from sklearn.cluster import DBSCAN

SENSITIVITY = 10 																					# the distance from occupied map nodes for a data point to be considered novel

def getyaw(q):																						# gets yaw from quaternion
	x, y, z, w = q.x, q.y, q.z, q.w
	return atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
	
def badtogood(x,y):																					# converts the bad frustrating ros coord system to the much nicer standard one
	return -1*y, x

def goodtobad(x,y):																					# converts good pure normal coord system to yucky bad ros coord system
	return y, -1*x

class Controller:
	def __init__(self):
		rospy.init_node("controller",anonymous=True)
		self.map = map.Map()
		self.map.display()
		rospy.Subscriber("scan",LaserScan,self.updateranges)
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)
		self.points=[]
		self.novelpoints=[]
		self.cluster = []
		self.pose = PoseWithCovarianceStamped().pose.pose
		self.x = self.y = self.i = 0
		

		self.posestamp = rospy.Time(0,0)
		self.rangestamp=rospy.Time(0,0)

	def updateranges(self,data):
		# Sync with pose
		self.rangestamp = rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs)
		if self.rangestamp > self.posestamp: 
			while self.rangestamp > self.posestamp: rospy.sleep(.01)

		# Calculate locations of data points
		self.points = []
		for r in range(len(data.ranges)):
			if data.ranges[r]==float('inf'): continue
			x, y = goodtobad(self.x, self.y)
			theta = getyaw(self.pose.orientation)+r*data.angle_increment
			if theta>pi: theta = -2*pi + theta
			h = data.ranges[r]

			self.points.append((badtogood(x+h*cos(theta), y+h*sin(theta)))) # final calculation
	
	def updatepose(self,msg):
		# Sync with ranges
		self.posestamp = rospy.Time(msg.header.stamp.secs,msg.header.stamp.nsecs)
		if self.posestamp > self.rangestamp: 
			while self.posestamp > self.rangestamp: rospy.sleep(.05)

		# Set pose variables
		self.x, self.y = badtogood(msg.pose.pose.position.x, msg.pose.pose.position.y)
		self.pose = msg.pose.pose

	def process(self):
		self.novelpoints = []
		pointscpy = deepcopy(self.points)
		for point in pointscpy:
			if self.map.getdistancefromnearestpoint(point)>SENSITIVITY:
				self.novelpoints.append(point)
		if len(self.novelpoints)>8: self.getclusters()

	def getclusters(self):
		start = time.time()
		formatted = numpy.array(self.novelpoints)
		res = DBSCAN(eps=.1,min_samples=3).fit(formatted)																	# epsilon is max distance for points to be neighbors, min_sampes is the number of samples for core points
		stop = time.time()
		self.cluster = list(res.labels_)


	def display(self):
		self.map.displaypose(self.x, self.y)
		self.map.displayscan(self.points,self.novelpoints,self.cluster)

if __name__=='__main__':
	C = Controller()
	print("Initialized.")
	while True:
		C.process()
		C.display()
		time.sleep(.5)
	#rospy.spin()
