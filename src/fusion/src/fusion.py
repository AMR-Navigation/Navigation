#!/usr/bin/env python

import rospy
from messages.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import message_filters

from time import * 
from math import *
from copy import deepcopy

from laser import *
from yolo_fov import *


class Fusion:
    #This function initilazes the ROS Node and sets up the LidarList, ObjectDetection subscribers. It aslo uses message filter to synch the lidar and YOLO detection messages with sync_callback
    def __init__(self):
        rospy.init_node("fusion", anonymous=False)
        self.laser_sub = message_filters.Subscriber("LidarList", UWOList)        #Laser subscriber
        self.detection_sub = message_filters.Subscriber("object_detection_results", detection)        #Detection subscriber
        self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatepose)    #Pose subscriber
        
        # Time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.laser_sub, self.detection_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.frame = rospy.Time.now()  # for synchronization purposes

        # Initialize pose data storage variables
        self.yaw = 0
        self.x = 0
        self.y = 0

        # Laser and detection variables
        self.laserarcs = []
        self.detectionarcs = []

        
	#This function is called when the synced messages are received from the subscribers. It calls updateLaser and updatedetections to proccess the data
    def sync_callback(self, lidar_data, detection_data):
        self.updatelaser(lidar_data)
        self.updatedetections(detection_data)
        #self.fuse()

    # Arc: (min, max) using the ROS rotation system, e.g. relative to vertical axis, counter-clockwise, and like
    #             0
    #        .5pi    -.5pi
    #            1
    def updatelaser(self,data):
        print("Got new laser data:")
        self.laserarcs = []
        for obj in data.objects:
            angles, coords = getarc(obj,self.yaw,self.x,self.y)
            self.laserarcs.append((angles, coords))
        #print rospy.Time(data.header.stamp.secs,data.header.stamp.nsecs), '|', self.frame
        print ("Laserarcs: ", self.laserarcs)

    # Arc: (fov, direction)
    def updatedetections(self,detection):
        pass
        print("Got new YOLO data")

        center_x = detection.x          #x-coordinate for center of the bounding box
        center_y = detection.y          #y-coordinate for center of the bounding box
        # width = detection.width
        # height = detection.height

        # center_x, center_y = box_center(x, y, width, height)
        self.detectionarcs.append(calc_angle(center_x, center_y))       #This array holds the horizontal and vertical angles for the center of the bouding box

        print("Detectionarcs: ", self.detectionarcs)
        


	#This function updates the robots current pose based on the amcl_pose topic.
    def updatepose(self,data):
        print("Got new pose data")
        # Set class variables to use in other callbacks
        self.yaw = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,
                                                             data.pose.pose.orientation.y,
                                                             data.pose.pose.orientation.z,
                                                             data.pose.pose.orientation.w])[2]
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def match_lidar_yolo(self):
        matched_objects = []
        for angles, coords in self.laserarcs:
            min_angle, max_angle = angles
            for detection_arc in self.detectionarcs:
                angle_x, angle_y = detection_arc
                #Check if YOLO detection angle is within Lidar arc
                if min_angle <= angle_x <= max_angle:
                    matched_objects.append(coords)     #Instead of storing the angles, I want to store the arcs matching coordinate points given to me by UWO list which are labeled mean.x and mean.y
                    break       #Once matched, break the inner loop
        return matched_objects
    
    def process_unmatched_lidar(self, matched_objects):
        matched_coords = [match[1] for match in matched_objects] #Extracting matched coordinate points
        unknown_coords = [coords for _, coords in self.laserarcs if coords not in matched_coords] #Instead of storing the arc of the lidar, I want to store the corresponding coordinate points(x and y) given to me by UWO list messages where it is stored as "mean"
        return unknown_coords
    

    def fuse(self):
        matched_objects = self.match_lidar_yolo()         
        print(matched_objects)                
        unknown_arcs = self.process_unmatched_lidar(matched_objects)    
        print    

    # Handle matched objects
    for arc, box in matched_objects:
        # Implement logic to update object locations based on arc and box
        print(f"Matched arc: {arc} with box: {box}")

    # Handle unknown objects
    for arc in unknown_arcs:
        # Implement logic to handle unknown arcs
        print(f"Unknown arc: {arc}")


        # CURRENT PLAN:
        # sync with detections and laser arcs
        # detectionarcs = self.detectionarcs
        # laserarcs = self.laserarcs
        # laserobjs = self.laserobjs
        # 
        # using previously compiled list as prior,
        # attempt to update the locations of each object
        #for i in range(len(self.record)):
            #for arc in laserarcs:

        # 
        # then take all remaining data and,
        # for every arc in laser arcs:
        #         attempt to identify it, if impossible then mark as unknown


if __name__ == "__main__":
    F = Fusion()
    print("Initialized.")
    rospy.spin()


# #Processes the YOLO detection data and apends the calculated angles to detectionarcs
    # def updatedetections2(self,detection):
    #     print("Got new YOLO data")
    #     print(detection.classification, self.yaw/pi, getarcfrombox(detection.x,detection.width, self.yaw)[0]/pi, getarcfrombox(detection.x,detection.width, self.yaw)[1]/pi)
    #     self.detections.append(detection)
    #     self.detectionarcs.append(getarcfrombox(detection.x,detection.width, self.yaw))