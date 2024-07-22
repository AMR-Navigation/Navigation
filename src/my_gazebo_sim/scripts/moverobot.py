#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class MultiRobot:
	def __init__(self):
		rospy.init_node('multirobot', anonymous=True)
		self.pub = rospy.Publisher('/tb3_2/move_base_simple/goal', PoseStamped, queue_size=10)
		self.status_sub = rospy.Subscriber('/tb3_2/move_base/status', GoalStatusArray, self.callback)
		self.toggle = 0
		self.goals=[(4,0),(7,1)]
	
	def togglegoal(self):
		self.toggle = (self.toggle+1)%len(self.goals)

	def sendgoal(self,g):
		x,y = g
		goal = PoseStamped()
		goal.header.frame_id = "map"
		goal.header.stamp = rospy.Time.now()
		
		goal.pose.position.x = x
		goal.pose.position.y = y
		goal.pose.position.z = 0.0
		
		goal.pose.orientation.x = 0.0
		goal.pose.orientation.y = 0.0
		goal.pose.orientation.z = 0.0
		goal.pose.orientation.w = 1.0
		
		rospy.loginfo("Sending goal...")
		for i in range(100000): self.pub.publish(goal)
		rospy.loginfo("goal sent.")

	def callback(self, data):
		for status in data.status_list:
			if status.status == 3:  # GoalStatus.SUCCEEDED
				rospy.loginfo("Goal reached!")
				self.togglegoal()
				self.sendgoal(self.goals[self.toggle])
				break

if __name__ == '__main__':
	try:
		multi = MultiRobot()
		multi.sendgoal(multi.goals[0])
		multi.togglegoal()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
