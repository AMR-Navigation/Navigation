#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import requests

APIKEY = open("key.txt", 'r').read()
APIURL = "https://detect.roboflow.com/hri-o1n8g/1?api_key=" + APIKEY

def predict(image_data, overlap=30, confidence=40, stroke=1, labels=False, _format="json"):
    session = requests.Session()
    headers = {"Content-Type": "application/x-www-form-urlencoded"}
    params = "&overlap={}&confidence={}&stroke={}&labels={}&format={}".format(
        str(overlap), str(confidence), str(stroke), str(labels), _format)
    resp = session.post(APIURL + params, headers=headers, data=image_data.encode("base64"))
    return resp.text

class ObjectDetectorNode:
    def __init__(self):
        rospy.init_node('object_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/object_detection_results', Image, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert cv_image to binary data
        _, img_encoded = cv2.imencode('.jpg', cv_image)
        img_bytes = img_encoded.tostring()

        # Perform object detection
        detection_result = predict(img_bytes)

        if detection_result is not None:
            # Log the detection result
            rospy.loginfo("Detection Result: {}".format(detection_result))

            # Publish the original image for visualization (optional)
            self.result_pub.publish(data)
        else:
            rospy.logwarn("Failed to get valid detection result.")

        # Debug: Print the API response directly
        rospy.loginfo("API Response: {}".format(detection_result))

if __name__ == '__main__':
    try:
        ObjectDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

