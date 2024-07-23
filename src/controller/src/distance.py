#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from math import sqrt

class Distance:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        rospy.init_node('distance', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.prev_x is not None and self.prev_y is not None:
            distance = sqrt((current_x - self.prev_x)**2 + (current_y - self.prev_y)**2)
            self.total_distance += distance

        self.prev_x = current_x
        self.prev_y = current_y

        rospy.loginfo("Total Distance Traveled: %f meters" % self.total_distance)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        distance = Distance()
        distance.run()
    except rospy.ROSInterruptException:
        pass
