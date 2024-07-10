import socketserver	
import struct
import base64
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from yolo_serv import *

HOST, PORT = "localhost", 9998
payload = -1


def getints(byte_string):
	int_list = []
	for i in range(0, len(byte_string), 4):
		# Unpack each 4-byte segment as a single unsigned int ('I')
		int_value = struct.unpack('<I', byte_string[i:i+4])[0]
		int_list.append(int_value)
	return int_list

class Server(socketserver.BaseRequestHandler):

	def handle(self):
		global payload
		if payload==-1: data = self.request.recv(1024).strip()
		else:
			data = self.request.recv(payload+1).strip()
			payload = -1
		print(len(data))
		
		if getints(data[:4])==[0]:			# prep message, contains payload length
			payload = getints(data)[1]
		else:								# inference
			imagenp = np.frombuffer(data[1:], np.uint8)
			imagearray = cv2.imdecode(imagenp, cv2.IMREAD_COLOR)
			print(imagearray.shape)
			quit()
			detections = process_image_frame(imagearray)
			print(detections)
			self.request.sendall(str(detections))

if __name__ == "__main__":
	with socketserver.TCPServer((HOST, PORT), Server) as server:
		server.serve_forever()