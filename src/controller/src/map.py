from Tkinter import *
from copy import deepcopy
from Queue import Queue
from time import time
from random import *
import os

RESOLUTION = .05
ORIGIN = (-7,-10)



class Map():
	def __init__(self):
		path = "~/map.pgm"
		self.mapfile = open(os.path.expanduser(path),'rb')
		self.setupgui()															# prepare tkinter stuff
		self.getmetadata()														# get info from map file
		self.setbegin()															# get the beginning position of the map data

		self.readmap()
		self.mapfile.close()

		

	def setupgui(self):
		self.frame = Tk()
		self.canvas = Canvas(self.frame, width=500, height=500,bg="white")
		self.canvas.pack()

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

	def getdistancefromnearestpoint(self,point):
		point = tuple(self.goodtomatrix(point[0],point[1]))
		visited = {}
		distances = {point: 0}
		q = Queue()
		q.put(point)

		# bfs
		while not q.empty():
			curr = q.get()
			if visited.get(curr)!=None or curr[0] < 0 or curr[0] > self.width or curr[1] < 0 or curr[1] > self.height: 
				continue   # not legal
			else: visited[curr] = True

			if self.map[curr[0]][curr[1]]==0:
				return distances[curr]									# if cell is occupied, return

			directions = [(-1, -1), (0, -1), (1, -1),   # Up-left, Left, Down-left
              (1, 0),  (1, 1),  (0, 1),    # Down, Down-right, Right
              (-1, 1), (-1, 0)]            # Up-right, Up

			for d in directions:
				neighbor = (curr[0]+d[0],curr[1]+d[1])
				q.put(neighbor)
				distances[neighbor] = distances[curr]+1
		return 999999999 # the map is empty lol



	def readmap(self):
		self.mapfile.seek(self.begin)
		self.map = []

		for i in range(self.height):
			self.map.append(list(ord(c) for c in self.mapfile.read(self.width)[::-1]))


	def display(self):
		self.canvas.delete("node")
		for i in range(len(self.map)):
			for j in range(len(self.map[i])):
				self.canvas.create_rectangle(
					(i,j),
					(i,j),
					fill= ("grey" if self.map[i][j]!=0x00 else "black"),
					width=0,
					tags="node")
		self.canvas.update()
		self.canvas.create_rectangle((self.goodtogui(0,0)),(self.goodtogui(0,0)),fill="blue",width = 3)

	def displaypose(self,x,y):
		self.canvas.delete("pose")
		self.canvas.create_rectangle(
			(self.goodtogui(x,y)),
			(self.goodtogui(x,y)),
			fill='red',
			outline='red',
			width=3,
			tags="pose"
		)
		self.canvas.update()

	def displayscan(self,points,npoints,cluster):
		self.canvas.delete("points")
		for point in points:
			if not point in npoints: self.canvas.create_rectangle(
				self.goodtogui(point[0],point[1]),
				self.goodtogui(point[0],point[1]),
				fill='lightgreen',
				width=0,
				tags="points"
			)
		colors = ["blue","red","purple","white"]
		for p in range(len(npoints)):
			point = npoints[p]
			self.canvas.create_rectangle(
				self.goodtogui(point[0],point[1]),
				self.goodtogui(point[0],point[1]),
				fill=colors[(cluster[p]%3 if cluster[p]!=-1 else 3)],
				width=0,
				tags="points"
			)
		self.canvas.update()






	def goodtogui(self,x,y):
		return x/RESOLUTION+float(self.width)/2 + ORIGIN[0], -1*y/RESOLUTION+float(self.height)/2 + ORIGIN[1]

	def goodtomatrix(self,x,y):						# note that this will round to the nearest node
		return int(round(x/RESOLUTION+float(self.width)/2 + ORIGIN[0])), int(round(-1*y/RESOLUTION+float(self.height)/2 + ORIGIN[1]))