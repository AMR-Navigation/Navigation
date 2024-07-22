#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import requests
import json
from messages.msg import detection

import client

# APIKEY = open("key.txt", 'r').read()[:-1]
# print '(', APIKEY, ')'
# APIURL = "https://detect.roboflow.com/hri-o1n8g/1?api_key=" + APIKEY

# def predict(image_data, overlap=30, confidence=40, stroke=1, labels=False, _format="json"):
# 	session = requests.Session()
# 	headers = {"Content-Type": "application/x-www-form-urlencoded"}
# 	params = "&overlap={}&confidence={}&stroke={}&labels={}&format={}".format(
# 		str(overlap), str(confidence), str(stroke), str(labels), _format)
# 	resp = session.post(APIURL + params, headers=headers, data=image_data.encode("base64"))
# 	return resp.text
	
class ObjectDetectorNode:
	def __init__(self):
		rospy.init_node('object_detector_node', anonymous=True)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.result_pub = rospy.Publisher('/object_detection_results', detection, queue_size=1)

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
		detection_result = client.inference(img_bytes)

		#print (detection_result)

		if detection_result is not None:
			# Parse the detection result
			predictions = json.loads(detection_result)
			for pred in predictions:
				detection_msg = detection()
				detection_msg.classification = pred.get('class', '')
				detection_msg.confidence = pred.get('confidence', 0.0)
				detection_msg.x = pred.get('x', 0.0)
				detection_msg.y = pred.get('y', 0.0)
				detection_msg.width = pred.get('width', 0.0)
				detection_msg.height = pred.get('height', 0.0)

				# Convert center coordinates to top-left coordinates
				x_center = pred.get('x', 0.0)
				y_center = pred.get('y', 0.0)
				width = pred.get('width', 0.0)
				height = pred.get('height', 0.0)
				x1 = x_center - (width / 2)
				y1 = y_center - (height / 2)
				x2 = x_center + (width / 2)
				y2 = y_center + (height / 2)

				# Draw bounding box on the image
				cv2.rectangle(cv_image, 
                              (int(x1), int(y1)), 
                              (int(x2), int(y2)), 
                              (0, 255, 0), 2)
				cv2.putText(cv_image, "{}: {:.2f}".format(pred['class'], pred['confidence']),
                            (int(x1), int(y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

				# Log the detection result
				rospy.loginfo("Class: {}, Confidence: {}, X: {}, Y: {}, Width: {}, Height: {}".format(
					detection_msg.classification, detection_msg.confidence, detection_msg.x, detection_msg.y,
					detection_msg.width, detection_msg.height))
				
				# Stamp the detection
				detection_msg.header.stamp = rospy.Time.now()
				detection_msg.header.frame_id = "detection"

				# Publish the detection result
				self.result_pub.publish(detection_msg)

				# Display the image with bounding boxes
				cv2.imshow('Object Detection', cv_image)
				cv2.waitKey(1)
			else:
				# Create empty detection
				detection_msg = detection()
				detection_msg.classification = "None"
				detection_msg.confidence = 0
				detection_msg.x = 0
				detection_msg.y = 0
				detection_msg.width = 0
				detection_msg.height = 0
				# Stamp the detection
				detection_msg.header.stamp = rospy.Time.now()
				detection_msg.header.frame_id = "detection"

				# Publish the detection result
				self.result_pub.publish(detection_msg)

		else:
			rospy.logwarn("Failed to get valid detection result.")

if __name__ == '__main__':
	try:
		ObjectDetectorNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
