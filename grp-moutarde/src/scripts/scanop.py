#!/usr/bin/python3

import math, rospy, random
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

img=np.array([[0,0,0]], np.uint8)

# Initialize ROS::node
rospy.init_node('training', anonymous=True)

object_cascade=cv2.CascadeClassifier()

def perception(data):
    img = np.array(list(data.data), np.uint8)
    img = np.resize(img, (data.height, data.width, 3))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    object=object_cascade.detectMultiScale(gray, scaleFactor=1.10, minNeighbors=3)
    #cv2.imshow("img", img)
    #cv2.waitKey(0)

commandListener = rospy.Subscriber("/camera/color/image_raw", Image, perception)

# Publish velocity commandes:
def move_command(data):
    pass
    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start training.py")
rospy.spin()