#!/usr/bin/env python2

import rospy
import threading
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

class MovePersonPlugin:
    def __init__(self):
        rospy.init_node('move_person_plugin')
        self.model_name = 'person_standing'
        self.max_distance = 5.0  # meters
        self.speed = 0.5  # m/s
        self.direction = 1
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
            distance_moved = abs(current_position.x - self.start_position.x)
            if distance_moved >= self.max_distance:
                self.direction *= -1

            new_position = Pose()
            new_position.position.x = current_position.x + self.direction * self.speed / 10.0
            new_position.position.y = current_position.y
            new_position.position.z = current_position.z

            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose = new_position

            self.set_model_state(model_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MovePersonPlugin()
    except rospy.ROSInterruptException:
        pass
