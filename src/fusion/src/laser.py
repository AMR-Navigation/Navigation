import rospy
from messages.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

from time import * 
from math import *
from copy import deepcopy

from laser import *


def badtogood(x,y):
	return -1*y, x

# Returns min and max angles of this cloud
def getarc(obj,yaw,x,y):
	x, y = badtogood(x, y)
	return -1*atan2(obj.points[0].x-x,obj.points[0].y-y), -1*atan2(obj.points[-1].x-x,obj.points[-1].y-y)
	#for p in obj.points:
	#	print p.x-x,p.y-y,": ",atan2(p.x-x,p.y-y)/pi
	#min_ = min(list(atan(point.y-y/point.x-x) for point in obj.points))
	#max_ = max(list(atan(point.y-y/point.x-x) for point in obj.points))
