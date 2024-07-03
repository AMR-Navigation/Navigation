#!/usr/bin/env python3

import cv2
import numpy as np
import requests
from inference_sdk import InferenceHTTPClient
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

# Initialize ROS node
rospy.init_node('turtlebot3_camera_inference')

# Initialize CvBridge
bridge = CvBridge()

# Roboflow Inference Client
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="1pKsd4B4m5E0PC4nKb1h"
)

def image_callback(ros_image):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Save the frame as a temporary file
    cv2.imwrite("/tmp/current_frame.jpg", cv_image)

    # Send the frame to Roboflow API for inference
    result = CLIENT.infer("/tmp/current_frame.jpg", model_id="hri-o1n8g/1")
    
    # Process the result (example: printing the result)
    print(result)

    # Optionally, display the frame (for debugging purposes)
    cv2.imshow("TurtleBot3 Camera", cv_image)
    cv2.waitKey(1)

# Subscribe to the TurtleBot3 camera topic
image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# Keep the program alive
rospy.spin()

# Release resources
cv2.destroyAllWindows()
