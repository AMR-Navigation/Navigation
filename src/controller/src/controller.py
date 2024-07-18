#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from messages.msg import *
import tf.transformations
from message_filters import Subscriber, TimeSynchronizer

import map

import time
from math import *
from copy import deepcopy
import numpy
from sklearn.cluster import DBSCAN

SENSITIVITY = 14 																					# the distance from occupied map nodes for a data point to be considered novel
MINPOINTSFORCLUSTER = 3																				# minimum number of points for a cluster to be published

def getyaw(q):																						# gets yaw from quaternion
	x, y, z, w = q.x, q.y, q.z, q.w
	return atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
	
def badtogood(x,y):																					# converts the bad frustrating ros coord system to the much nicer standard one
	return -1*y, x

def goodtobad(x,y):																					# converts good pure normal coord system to yucky bad ros coord system
	return y, -1*x

def getmeanpoint(points):																			# argument must be a list of the messages.msg.point type
	p = point()
	p.x = 0.
	p.y = 0.
	for point_ in points:
		p.x += point_.x
		p.y += point_.y
	p.x /= len(points)
	p.y /= len(points)
	return p

class Controller:
	def __init__(self):
		rospy.init_node("controller",anonymous=True)
		self.map = map.Map()
		self.map.display()
		self.lasersub = Subscriber("scan",LaserScan)
		self.posesub = Subscriber("amcl_pose", PoseWithCovarianceStamped)
		self.ts = TimeSynchronizer([self.lasersub, self.posesub], 10)
		self.ts.registerCallback(self.updaterangesandpose)
		self.publisher = rospy.Publisher('LidarList', UWOList, queue_size = 8)
		self.points=[]
		self.novelpoints=[]
		self.cluster = []
		self.pose = PoseWithCovarianceStamped().pose.pose
		self.x = self.y = self.i = 0
		

		self.posestamp = rospy.Time(0,0)
		self.rangestamp=rospy.Time(0,0)

	def updaterangesandpose(self,rangemsg,posemsg):
		ranges = deepcopy(rangemsg)
		msg = deepcopy(posemsg)

		# Set pose variables
		self.x, self.y = badtogood(msg.pose.pose.position.x, msg.pose.pose.position.y)
		self.pose = msg.pose.pose

		self.points = []
		for r in range(len(ranges.ranges)):
			if ranges.ranges[r]==float('inf'): continue
			x, y = goodtobad(self.x, self.y)
			theta = getyaw(self.pose.orientation)+r*ranges.angle_increment
			if theta>pi: theta = -2*pi + theta
			h = ranges.ranges[r]

			self.points.append((badtogood(x+h*cos(theta), y+h*sin(theta)))) # final calculation

		

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
		if len(self.novelpoints)>6: 
			self.getclusters()
			self.publishclusters()
		else:
			self.novelpoints = []

	def getclusters(self):
		formatted = numpy.array(self.novelpoints)
		res = DBSCAN(eps=.12,min_samples=3).fit(formatted)																	# epsilon is max distance for points to be neighbors, min_sampes is the number of samples for core points
		self.cluster = list(res.labels_)

	def publishclusters(self):
		# Create the list of clusters
		uwolist = UWOList()
		clust = deepcopy(self.cluster)																						# avoid threading issues
		novels = deepcopy(self.novelpoints)
		for i in range(len(novels)):
			if clust[i]==-1: continue																				# point is noise, skip it
			while clust[i]>len(uwolist.objects)-1: uwolist.objects.append(UWO())										# add a new UWO
			npoint = point()																								# create the point
			npoint.x = novels[i][0]
			npoint.y = novels[i][1]
			uwolist.objects[clust[i]].points.append(npoint)															# properly add the point

		# Filter the list and set the means
		toremove = []
		for uwo in uwolist.objects:
			if len(uwo.points)<MINPOINTSFORCLUSTER: toremove.append(uwo)
			else: uwo.mean = getmeanpoint(uwo.points)
		for uwo in toremove: uwolist.objects.remove(uwo)

		# Stamp it
		uwolist.header.stamp = rospy.Time.now()
		uwolist.header.frame_id = "uwolist"

		# Send it
		self.publisher.publish(uwolist)





	def display(self):
		self.map.displaypose(self.x, self.y)
		P = deepcopy(self.points)
		NP = deepcopy(self.novelpoints)
		C = deepcopy(self.cluster)
		self.map.displayscan(P,NP,C)

if __name__=='__main__':
	C = Controller()
	print("Initialized.")
	while True:
		C.process()
		C.display()
		time.sleep(.5)
	#rospy.spin()
