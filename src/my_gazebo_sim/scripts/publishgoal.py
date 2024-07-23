#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

GOAL = (4.5,8.5) 														# x,y; ros coords

class PublishGoal:
	def __init__(self):
		rospy.init_node('publishgoal', anonymous=True)
		self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
		self.status_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback)
		self.done = False

	def sendgoal(self):
		while not self.done:
			goal = PoseStamped()
			goal.header.frame_id = "map"
			goal.header.stamp = rospy.Time.now()
			
			goal.pose.position.x = GOAL[0]
			goal.pose.position.y = GOAL[1]
			goal.pose.position.z = 0.0
			
			goal.pose.orientation.x = 0.0
			goal.pose.orientation.y = 0.0
			goal.pose.orientation.z = 0.0
			goal.pose.orientation.w = 1.0
			
			self.pub.publish(goal)
		exit(0)

	def callback(self, data):
		self.done = True

if __name__ == '__main__':
	try:
		pb = PublishGoal()
		rospy.loginfo("Sending goal...")
		pb.sendgoal()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
