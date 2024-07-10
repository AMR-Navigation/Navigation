#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from Tkinter import *
from copy import deepcopy

RESOLUTION = .05


class Parser():
	def __init__(self):
		self.mapfile = open("/home/dan/map.pgm",'rb')
		self.frame = Tk()
		self.canvas = Canvas(self.frame, width=500, height=500,bg="white")
		self.canvas.pack()
		self.pose = PoseWithCovarianceStamped().pose.pose
		self.marks = [(0,0)]

		self.getmetadata()
		
		# get point of entry
		self.setbegin()

		self.map = []
		self.dim = int(4/RESOLUTION)

		# subscribe
		rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.updatemap)
		
		""""""

	def getmetadata(self):
		b = self.mapfile.read(50)
		b = b.decode('utf-8').split('\n')
		self.width = int(b[2].split(' ')[0])
		self.height = int(b[2].split(' ')[1])
		self.gradient = int(b[3])
		# TODO: Get resolution from yaml file

	def setbegin(self):
		self.mapfile.seek(0)
		c = 0
		linebreaks=0
		while linebreaks<4:
			linebreaks+= (1 if self.mapfile.read(1)==b'\n' else 0)
			c+=1
		self.begin = c

	def initmap(self):
		pass
	
	def updatemap(self, msg):
		self.pose = msg.pose.pose
		x, y = self.trans(msg.pose.pose.position.x, msg.pose.pose.position.y)

		minx = int(max(x-self.dim/2,0))
		maxx = int(min(x+self.dim/2,self.width))
		miny = int(max(y-self.dim/2,0))
		maxy = int(min(y+self.dim/2,self.height))
		d = maxx-minx
		r = maxy-miny

		#print minx, ' ', maxx, ' ', miny, ' ', maxy, ' ', r, ' ', d

		self.map = []		# temp
		self.mapfile.seek(self.begin+minx*self.width)
		for i in range(d):
			self.mapfile.seek(self.width-maxy,1)
			v = list(ord(c) for c in self.mapfile.read(r))[::-1]
			self.mapfile.seek(self.width-(self.width-miny),1)
			"""v = []
			for j in range(r):
				self.mapfile.seek(minx+i,1)
				b = ord(self.mapfile.read(1))
				v.append(b)
				self.mapfile.seek(self.width-(minx+i+1),1)"""
			self.map.append(v)
			#print(v)

	def display(self):
		self.canvas.delete("point")
		self.displaymap()
		self.displaypose()
		self.displaymarks()
		self.canvas.update()

	def displaymap(self):
		cpy = deepcopy(self.map)
		for i in range(len(cpy)):
			for j in range(len(cpy[i])):
				self.canvas.create_rectangle(
					(50+i,50+j),
					(50+i,50+j),
					fill= ("grey" if cpy[i][j]!=0x00 else "black"),
					width=0,
					tags="point")

	def displaypose(self):
		;
		self.canvas.create_rectangle(
			(self.transtogui(self.pose.position.x, self.pose.position.y)),
			(self.transtogui(self.pose.position.x, self.pose.position.y)),
			fill="red",
			outline="red",
			width=5,
			tags="point"
		)

	def displaymarks(self):
		for mark in self.marks:
			self.canvas.create_rectangle(
				self.transtogui(mark[0],mark[1]),
				self.transtogui(mark[0],mark[1]),
				fill="lightgreen",
				width=0,
				tags="point"
			)

	def trans(self,x,y):																				# translates ros coords into map coords where x-> and yv
		return (-1*y)/RESOLUTION  + self.width/2, (-1*x)/RESOLUTION + self.height/2
	
	def transtogui(self,x,y):																			# translates ros coords into gui pixel coords
		newx, newy = self.trans(x,y)
		myx, myy = self.trans(self.pose.position.x, self.pose.position.y) 
		return 50+(newx-myx+self.dim/2), 50+(newy-myy+self.dim/2)




	def mark(self, range, angle):
		self.marks.append()

		

if __name__=="__main__":
	rospy.init_node("parser")
	P = Parser()
	P.frame.mainloop()
