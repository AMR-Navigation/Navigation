import math
from math import pi
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

#Camera specs
image_width = 640  # pixels
image_height = 480  # pixels
focal_length_x = 530.4669406576809 # pixels
principal_point_x = 320.5 # pixels
principal_point_y = 240.5 # pixels
# yolo_width = 640
# horizontal_fov = 1.0856  # radians
# vertical_fov = 0.8632  # radians



#Focal length in pixels
# focal_length_x = (image_width / 2) / math.tan(math.radians(horizontal_fov / 2))
# focal_length_y = (image_height / 2) / math.tan(math.radians(vertical_fov / 2))

#Center of the image
# principal_point_x = image_width / 2
# principal_point_y = image_width / 2

# def box_center(x,y,width,height):
#     center_x = x 
#     center_y = y 
#     return x, y

#This function calculates the angle for the center of the bouding box relative to the camera's optical axis.
def calc_angle(yaw, center_x, center_y):

    global principal_point_x, principal_point_y, focal_length_x, focal_length_y
    
    angle_x = (center_x - principal_point_x) / focal_length_x		# horizontal angle relative to optical azis
    #angle_y = math.atan2((center_y - principal_point_y), focal_length_y)            #vertical angle relative to optical axis
	
    hor_angle = math.atan(angle_x)
    return (yaw - hor_angle),0



# def modulate(theta1,theta2):
# 	return (2*pi + theta1 if theta1<-1*pi else (-2*pi +theta1 if theta1>pi else theta1)),(2*pi + theta2 if theta2<-1*pi else (-2*pi +theta2 if theta2>pi else theta2))

# def getarcfrombox(x, width, yaw):
# 	p1, p2 = (x-width/2)/yolo_width*image_width, (x+width/2)/yolo_width*image_width
# 	theta1, theta2 = -1*((p1/image_width)*math.radians(horizontal_fov) - math.radians(horizontal_fov)/2), -1*((p2/image_width)*math.radians(horizontal_fov) - math.radians(horizontal_fov)/2)
# 	return modulate(yaw+theta1, yaw+theta2)