import socketserver	
import struct
import base64
import json

from yolo_serv import *

HOST, PORT = "localhost", 9999

class Server(socketserver.BaseRequestHandler):
	def handle(self):
		imagebuffer = []							# each packet will be put in here
		while True:
			data = self.request.recv(1024)			# get a packet
			if data==b"0x00": break					# this means all the image data has been sent, so break out of loop

			imagebuffer.append(data)
			self.request.sendall("Ok".encode('utf-8'))							# respond
		print("Received ",len(b"".join(imagebuffer))," bytes")

		# Send the completed image to the model
		image = base64.b64decode(b"".join(imagebuffer))							# decode it
		nparr = np.frombuffer(image, np.uint8)									# make it a numpy array
		frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)							# encode that array
		identification = json.dumps(process_image_frame(frame))					# process the frame and make the response json
		self.request.sendall(identification.encode('utf-8'))					# send the json back to the client
		print("Done")
		














if __name__ == "__main__":
	with socketserver.TCPServer((HOST, PORT), Server) as server:
		server.serve_forever()