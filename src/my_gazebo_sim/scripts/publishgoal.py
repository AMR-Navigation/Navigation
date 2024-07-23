#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

GOAL = (0,0) 														# x,y; ros coords

class PublishGoal:
	def __init__(self):
		rospy.init_node('publishgoal', anonymous=True)
		self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

	def sendgoal(self):
		x,y = g
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
		
		rospy.loginfo("Sending goal...")
		self.pub.publish(goal)
		rospy.loginfo("goal sent.")

if __name__ == '__main__':
	try:
		pb = PublishGoal()
		pb.sendgoal()
	except rospy.ROSInterruptException:
		pass
