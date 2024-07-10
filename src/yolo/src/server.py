import socketserver	
import struct
import base64
import json

from yolo_serv import *

HOST, PORT = "localhost", 9999

class Server(socketserver.BaseRequestHandler):
	def handle(self):
		imagebuffer = []
		while True:
			data = self.request.recv(1024)
			if data==b"0x00": break
			print(len(data))

			imagebuffer.append(data)
			self.request.sendall("Ok".encode('utf-8'))
		print("Received ",len(b"".join(imagebuffer))," bytes")

		image = base64.b64decode(b"".join(imagebuffer))
		nparr = np.frombuffer(image, np.uint8)
		frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
		identification = json.dumps(process_image_frame(frame))
		self.request.sendall(identification.encode('utf-8'))
		print("Done")
		


if __name__ == "__main__":
	with socketserver.TCPServer((HOST, PORT), Server) as server:
		server.serve_forever()