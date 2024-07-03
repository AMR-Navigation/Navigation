#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class Controller:
	def __init__(self):
		rospy.init_node("controller",anonymous=True)
		rospy.Subscriber("scan",LaserScan,self.callback)
		rospy.spin()

	def callback(self,data):
		rospy.loginfo(rospy.get_caller_id() + str(data.ranges))

if __name__=='__main__':
	C = Controller()
