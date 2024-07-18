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
        self.speed = 0.1  # m/s
        self.rate = rospy.Rate(10)  # 10 Hz

        # Wait for services to become available
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Initialize waypoints and current index
        self.start_position = self.get_current_position()
        self.define_waypoints()
        self.current_waypoint_index = 0

        # Start movement thread
        self.thread = threading.Thread(target=self.move_model)
        self.thread.start()

    def define_waypoints(self):
        # Define waypoints relative to the start position
        self.waypoints = [
            (self.start_position.x + 4.0, self.start_position.y),  # Move forward 4 meters
            (self.start_position.x + 4.0, self.start_position.y - 3.0),  # Move to the left 5 meters
            (self.start_position.x + 7.0, self.start_position.y - 3.0),
            (self.start_position.x + 4.0, self.start_position.y),
            (self.start_position.x, self.start_position.y)

            # (self.start_position.x + 9.0, self.start_position.y - 5.0),  # Move right 9 meters
            # # Additional waypoints can be added here
        ]

    def get_current_position(self):
        try:
            model_state = self.get_model_state(self.model_name, '')
            return model_state.pose.position
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return Pose().position  # Return a default position in case of an error

    def move_model(self):
        while not rospy.is_shutdown():
            current_position = self.get_current_position()
            target_position = self.waypoints[self.current_waypoint_index]

            distance_to_target = ((target_position[0] - current_position.x) ** 2 + 
                                  (target_position[1] - current_position.y) ** 2) ** 0.5

            if distance_to_target < 0.1:
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
                target_position = self.waypoints[self.current_waypoint_index]

            direction_x = (target_position[0] - current_position.x) / distance_to_target
            direction_y = (target_position[1] - current_position.y) / distance_to_target

            new_position = Pose()
            new_position.position.x = current_position.x + direction_x * self.speed / 10.0
            new_position.position.y = current_position.y + direction_y * self.speed / 10.0
            new_position.position.z = current_position.z  # Assuming no movement in z direction

            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose = new_position

            try:
                self.set_model_state(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        MovePersonPlugin()
    except rospy.ROSInterruptException:
        pass
