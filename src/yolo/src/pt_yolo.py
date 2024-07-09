#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import socket
import json
import base64
from messages.msg import detection

class ObjectDetectorNode:
    def __init__(self):
        rospy.init_node('object_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/object_detection_results', detection, queue_size=1)
        self.server_address = ('<SERVER_IP>', 5000)  # Replace <SERVER_IP> with the server's IP address

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Encode image as JPEG
        _, img_encoded = cv2.imencode('.jpg', cv_image)
        img_bytes = img_encoded.tostring()
        img_base64 = base64.b64encode(img_bytes).decode('utf-8')

        # Send image to the server and receive detection results
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect(self.server_address)
            client_socket.send(img_base64.encode('utf-8'))
            
            response = client_socket.recv(1024)
            detection_result = json.loads(response.decode('utf-8'))
            client_socket.close()
        except Exception as e:
            rospy.logerr("Socket error: {}".format(e))
            return

        detections = detection_result.get('detections', [])

        for pred in detections:
            detection_msg = detection()
            detection_msg.classification = pred.get('class', '')
            detection_msg.confidence = pred.get('confidence', 0.0)
            detection_msg.x = pred.get('x', 0.0)
            detection_msg.y = pred.get('y', 0.0)
            detection_msg.width = pred.get('width', 0.0)
            detection_msg.height = pred.get('height', 0.0)

            # Log the detection result
            rospy.loginfo("Class: {}, Confidence: {}, X: {}, Y: {}, Width: {}, Height: {}".format(
                detection_msg.classification, detection_msg.confidence, detection_msg.x, detection_msg.y,
                detection_msg.width, detection_msg.height))

            # Publish the detection result
            self.result_pub.publish(detection_msg)

if __name__ == '__main__':
    try:
        ObjectDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
