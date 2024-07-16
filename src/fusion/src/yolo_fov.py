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


#This function calculates the angle for the center of the bouding box relative to the camera's optical axis.
def calc_angle(yaw, center_x, center_y):

    global principal_point_x, principal_point_y, focal_length_x, focal_length_y
    
    angle_x = (center_x - principal_point_x) / focal_length_x		# horizontal angle relative to optical azis
    
	
    hor_angle = math.atan(angle_x)
    return (yaw - hor_angle),0.0


