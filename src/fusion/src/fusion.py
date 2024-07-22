#!/usr/bin/env python

import rospy
from messages.msg import *
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import tf.transformations
import message_filters
import numpy as np

from time import * 
from math import *
from copy import deepcopy

from laser import *
from yolo_fov import *


class Fusion:
    # This function initializes the ROS Node and sets up the LidarList, ObjectDetection subscribers.
    # It also uses message filter to sync the lidar and YOLO detection messages with sync_callback.
    def __init__(self, threshold=0.1):
        rospy.init_node("fusion", anonymous=False)
        self.laser_sub = message_filters.Subscriber("LidarList", UWOList)  # Laser subscriber
        self.detection_sub = message_filters.Subscriber("object_detection_results", detection)  # Detection subscriber
        self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)  # Pose subscriber

        self.pub = rospy.Publisher("coordinates_topic", objectsList, queue_size=10)
        
        # Time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.laser_sub, self.detection_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.frame = rospy.Time.now()  # for synchronization purposes

        # Initialize pose data storage variables
        self.yaw = 0
        self.x = 0
        self.y = 0

        self.directions = []                    # look up by arc index

        # Laser and detection variables
        self.laserarcs = []
        self.detectionarcs = []

        # Initialize the threshold for distance comparison
        self.threshold = threshold

    # This function is called when the synced messages are received from the subscribers.
    # It calls updateLaser and updateDetections to process the data.
    def sync_callback(self, lidar_data, detection_data):
        self.updatelaser(lidar_data)
        self.updatedetections(detection_data)
        self.fuse()

    # Arc: (min, max) using the ROS rotation system, e.g. relative to vertical axis, counter-clockwise, and like
    #              0
    #        .5pi    -.5pi
    #              1
    def updatelaser(self, data):
        print("Got new laser data:")
        self.laserarcs = []
        self.directions = []
        for obj in data.objects:
            angles, coords = getarc(obj, self.yaw, self.x, self.y)
            self.laserarcs.append((angles, coords))
            self.directions.append(obj.direction)
            print("Coordinates: ", coords)
            print("Laser Angles: ", angles)

    # Arc: (fov, direction)
    def updatedetections(self, detection):
        print("Got new YOLO data")
        if detection.classification=="None": return
        center_x = detection.x  # x-coordinate for center of the bounding box
        center_y = detection.y  # y-coordinate for center of the bounding box
        classification = detection.classification
        angle_x, angle_y = calc_angle(self.yaw, center_x, center_y)
        self.detectionarcs.append((classification, angle_x, angle_y))  # This array holds the horizontal and vertical angles for the center of the bounding box
        print("Bounding Box angle: ", self.detectionarcs)

    # This function updates the robot's current pose based on the amcl_pose topic.
    def updatepose(self, data):
        # print("Got new pose data")
        self.yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,
                                                             data.pose.pose.orientation.y,
                                                             data.pose.pose.orientation.z,
                                                             data.pose.pose.orientation.w])[2]
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def match_lidar_yolo(self):
        matched_objects = []
        i=0
        for angles, coords in self.laserarcs:
            min_angle, max_angle = angles
            for classification, angle_x, angle_y in self.detectionarcs:
                # Check if YOLO detection angle is within Lidar arc with a standard deviation of +-0.30 radians
                if min_angle - 0.30 <= angle_x <= max_angle + 0.30:
                    matched_objects.append((classification, coords, self.directions[i]))
                    break  # Once matched, break the inner loop
            i+=1

        print("Printing Matched Objects: ", matched_objects)
        return matched_objects

    def process_unmatched_lidar(self, matched_objects):
        # Extracting matched coordinate points
        matched_coords = [match[1] for match in matched_objects]
        print(matched_coords)
        # # Filter out the matched coordinates from laserarc
        unknown_coords = [coords for _, coords in self.laserarcs if coords not in matched_coords]
        unknowndirs = [self.directions[i] for i in range(len(self.laserarcs)) if self.laserarcs[i][1] not in matched_coords]    # LMAO
        
        # Remove coordinates where both x and y have to be at most 0.6 away from any other coordinate
        filtered_unmatched_coords = []
        self.tempstorageforunmatcheddirections = []
        i=0
        for coord in unknown_coords:
            if not any(self.is_close(coord, existing, threshold=0.3) for existing in filtered_unmatched_coords):
                filtered_unmatched_coords.append(coord)
                self.tempstorageforunmatcheddirections.append(unknowndirs[i])
            else:
                print("Filtered out coord", coord, "as it's close to an existing coord in filtered list")
            i+=1

        
        return filtered_unmatched_coords

    def is_close(self, coord1, coord2, threshold):
        # print("Checking if ", coord1, " is close to ", coord2)
        return abs(coord1[0] - coord2[0]) < threshold and abs(coord1[1] - coord2[1]) < threshold

    def fuse(self):
        matched_objects = self.match_lidar_yolo()         
        print("Matched Objects", matched_objects)                
        unmatched_objects = self.process_unmatched_lidar(matched_objects)    
        print("Unmatched Objects", unmatched_objects)   

        msg = objectsList()
        msg.matched_objects = []
        msg.unmatched_objects = []

        for classification, coord, direction in matched_objects:
            coord_x, coord_y = coord
            obj = objects()
            obj.classification = classification
            obj.x = coord_x
            obj.y = coord_y
            obj.direction = direction
            # obj.coordinates = [Point(x=coord_x, y=coord_y, z=0)]
            # obj.x = coord[0]
            # obj.y = coord[1]

            msg.matched_objects.append(obj)
        
        i=0
        for coord in unmatched_objects:
            coord_x, coord_y = coord
            obj = objects()
            obj.classification = 'unknown'
            obj.x = coord_x
            obj.y = coord_y
            obj.direction = self.tempstorageforunmatcheddirections[i]
            # obj.coordinates = [Point(x=coord_x, y=coord_y, z=0)]
            msg.unmatched_objects.append(obj)
            i+=1
        # msg.unmatched_objects = [Point(x=coord[0], y=coord[1], z=0) for coord in unmatched_objects]
        # for classification, coord in unmatched_objects:
        #     obj = objects()
        #     obj.classification = "unknown"
        #     obj.coordinates = [Point(x=coord[0], y=coord[1], z=0)]
        #     # obj.x = coord[0]
        #     # obj.y = coord[1]
        #     msg.unmatched_objects.append(obj)

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "objectsList"

        self.pub.publish(msg)
        rospy.loginfo("Published matched and unmatched objects to coordinate_topic")


if __name__ == "__main__":
    F = Fusion(threshold=0.1)
    print("Initialized.")
    rospy.spin()
