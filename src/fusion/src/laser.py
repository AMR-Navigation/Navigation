import rospy
from messages.msg import *

from time import * 
from math import atan2


def badtogood(x,y):
	return -1*y, x

# Returns min and max angles of this cloud
def getarc(obj,yaw,x,y):
	x, y = badtogood(x, y)
	min_angle = -1*atan2(obj.points[0].x-x,obj.points[0].y-y)
	max_angle = -1*atan2(obj.points[-1].x-x,obj.points[-1].y-y)
	coord_x = obj.mean.x
	coord_y = obj.mean.y
	return (min_angle, max_angle), (coord_x,coord_y)


def getanglefrompoint(x,y,robotx,roboty):
	return -1*atan2(x-robotx, y-roboty)