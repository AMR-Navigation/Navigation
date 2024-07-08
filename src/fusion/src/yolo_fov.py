import math
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

#Camera specs
image_width = 3280  # pixels
image_height = 2464  # pixels
horizontal_fov = 62.2  # degrees
vertical_fov = 48.8  # degrees

#Focal length in pixels
focal_length_x = (image_width / 2) / math.tan(math.radians(horizontal_fov / 2))
focal_length_y = (image_height / 2) / math.tan(math.radians(vertical_fov / 2))

#Center of the image
principal_point_x = image_width / 2
principal_point_y = image_width / 2

def box_center(x,y,width,height):
    center_x = x + width / 2
    center_y = y + height / 2
    return center_x, center_y

def calc_angle(center_x, center_y, principal_point_x, principal_point_y, focal_length_x, focal_length_y):
    angle_x = math.atan2((center_x - principal_point_x), focal_length_x)
    angle_y = math.atan2((center_y - principal_point_y), focal_length_y)
    return math.degrees(angle_x), math.degrees(angle_y)
