#!/usr/bin/env python

#import rospy
#from std_msgs.msg import String
from tkinter import *

class Parser():
	def __init__(self):
		self.map = open("/home/dan/map.pgm",'rb')
		self.frame = Tk()
		self.canvas = Canvas(self.frame, width=500, height=500)
		self.canvas.pack()

		self.getmetadata()
		
		# get point of entry
		self.map.seek(0)
		count = 0
		while count<4: count+= (1 if self.map.read(1)==b'\n' else 0)
		
		for i in range(self.height):
			for j in range(self.width):
				self.canvas.create_rectangle(
					(50+j,50+i),
					(50+j,50+i),
					fill='#'+hex(int((255/self.gradient) * int.from_bytes(self.map.read(1),'little')))[2:].zfill(2)*3,
					width=0)
			self.canvas.update()
		

	def getmetadata(self):
		b = self.map.read(50)
		b = b.decode('utf-8').split('\n')
		self.width = int(b[2].split(' ')[0])
		self.height = int(b[2].split(' ')[1])
		self.gradient = int(b[3])


if __name__=="__main__":
	P = Parser()
	P.frame.mainloop()

