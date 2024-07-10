import socketserver	
import struct
import base64
import json
from time import *

from yolo_serv import *

HOST, PORT = "localhost", 9999

class Server(socketserver.BaseRequestHandler):
	def handle(self):
		start = time()
		imagebuffer = []							# each packet will be put in here
		while True:
			data = self.request.recv(1024)			# get a packet
			if data==b"0x00": break					# this means all the image data has been sent, so break out of loop

			imagebuffer.append(data)
			self.request.sendall("Ok".encode('utf-8'))							# respond
		stop=time()
		print("Received ",len(b"".join(imagebuffer))," bytes in ",stop - start," seconds")
	
		# Send the completed image to the model
		start = time()
		image = base64.b64decode(b"".join(imagebuffer))							# decode it
		nparr = np.frombuffer(image, np.uint8)									# make it a numpy array
		frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)							# encode that array
		stop = time()
		identification = json.dumps(process_image_frame(frame))					# process the frame and make the response json
		self.request.sendall(identification.encode('utf-8'))					# send the json back to the client
		
		print("Done in ",stop-start," additional seconds\n")
		














if __name__ == "__main__":
	while True:
		try:
			with socketserver.TCPServer((HOST, PORT), Server) as server:
				server.serve_forever()
		except OSError as e:
			PORT-=1
			print("Gos OSError. Changing port to ",PORT)