import socket
import struct

HOST, PORT = "localhost", 9998

def getbyte(i): # little endian
	return struct.pack('<I', i)

def getints(byte_string):
	int_list = []
	for i in range(0, len(byte_string), 4):
		# Unpack each 4-byte segment as a single unsigned int ('I')
		int_value = struct.unpack('<I', byte_string[i:i+4])[0]
		int_list.append(int_value)
	return int_list

def inference(image):
	#image = image.encode("base64")
	print len(image)
	payload = len(image)
	prep(payload)

	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((HOST, PORT))
	sock.sendall(image)
	resp = sock.recv(1024).decode("utf-8")

	print("Received: {}".format(resp))

def prep(l):
	# Create socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((HOST, PORT))

	# Prepare message
	msg = getbyte(0)+getbyte(l)

	sock.sendall(msg)

if __name__=="__main__":
	file = open("test.jpg",'rb')
	inference(file.read())
	file.close()