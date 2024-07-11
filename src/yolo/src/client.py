import socket
import struct

HOST, PORT = "localhost", 9999

def inference(image):
	image = image.encode("base64")
	print "Sending ",len(image)," bytes"

	# Connect to the socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((HOST, PORT))

	# Send all the data as packets of length 1024
	i=1024
	while i<len(image):							# loop through in blocks of 1024
		sock.sendall(image[i-1024:i])
		resp = sock.recv(1024).decode("utf-8")
		if resp!="Ok": print "ERROROROROROROOR"
		i+=1024
	sock.sendall(image[i-1024:])				# send the remaining data
	resp = sock.recv(1024).decode("utf-8")
	if resp!="Ok": print "ERROROROROROROOR"		
	sock.sendall(b"0x00")						# signal to terminate

	# Receive classification
	classification = sock.recv(1024).decode("utf-8")
	return classification


























if __name__=="__main__":
	file = open("person.jpg",'rb')
	print inference(file.read())
	file.close()