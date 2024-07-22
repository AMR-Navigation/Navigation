#!/usr/bin/env python2

import rospy
import threading
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from math import *


def isabout(a,b):
	if abs(a-b) < .2:
		return True
	return False

class TestPersonPlugin:
	def __init__(self):
		rospy.init_node('test_person_plugin')
		self.model_name = 'person_standing'
		self.turn_distance1 = 3.5  # meters
		self.turn_distance2 = 10  # meters
		self.turn_distance3 = 16.5  # meters
		self.cycle = 20
		self.speed = .2  # m/s
		self.direction = (-1,0)
		self.rate = rospy.Rate(10)  # 10 Hz

		rospy.wait_for_service('/gazebo/get_model_state')
		rospy.wait_for_service('/gazebo/set_model_state')
		self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		self.start_position = self.get_current_position()

		self.thread = threading.Thread(target=self.move_model)
		self.thread.start()

	def get_current_position(self):
		model_state = self.get_model_state(self.model_name, '')
		return model_state.pose.position
	
	def move_model(self):
		while not rospy.is_shutdown():
			current_position = self.get_current_position()
			distance_moved = sqrt((current_position.x - self.start_position.x)**2 + (current_position.y - self.start_position.y)**2)
			if isabout(distance_moved%self.cycle,self.turn_distance1):
				self.direction = (0,-1)
			elif isabout(distance_moved%self.cycle,self.turn_distance2):
				self.direction = (0,1)
			elif isabout(distance_moved%self.cycle,self.turn_distance3):
				self.direction = (1,0)
			elif isabout(distance_moved%self.cycle,0):
				self.direction = (-1,0)

			new_position = Pose()
			new_position.position.x = current_position.x + self.direction[0] * self.speed / 10.0
			new_position.position.y = current_position.y + self.direction[1] * self.speed / 10.0
			new_position.position.z = current_position.z

			model_state = ModelState()
			model_state.model_name = self.model_name
			model_state.pose = new_position

			self.set_model_state(model_state)
			print("moved")
			self.rate.sleep()

if __name__ == '__main__':
	try:
		TestPersonPlugin()
	except rospy.ROSInterruptException:
		pass
