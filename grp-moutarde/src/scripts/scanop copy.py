#!/usr/bin/python3

import math, rospy, random
import numpy as np
import cv2
import kobuki_led_controller
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

img=np.array([[200,33,213]], np.uint8)

# Initialize ROS::node
rospy.init_node('training', anonymous=True)
# chemin = rospy.__path__  #+ "src/scripts/cascade.xml"
# print(chemin)
#bridge = CvBridge()
object_cascade=cv2.CascadeClassifier("/home/grp-moutarde/catkin_ws/src/larm_moutarde/grp-moutarde/src/scripts/cascade1_c.xml")

#clignote = False
# blinker = kobuki_led_controller.LedBlinker()
# blinker.start()

# def blink(data):
#     blinker.set_on_error()
#     rospy.sleep(2)
#     blinker.set_on_ok
#     rospy.sleep(2)
#     blinker.set_on_off

def perception_img(data):
    #img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    object=object_cascade.detectMultiScale(gray, 1.2, minNeighbors=3)
    for (x,y,w,h) in object:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255),2)
        #rospy.Timer(rospy.Duration(2),blink,oneshot=True)
    # cv2.imshow("img", img)
    # cv2.waitKey(33)

def perception_depth(data):
    depth = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    cv2.imshow("depth", depth)
    cv2.waitKey(33)



commandListener_img = rospy.Subscriber("/camera/color/image_raw", Image, perception_img)
commandListener_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, perception_depth)

# Publish velocity commandes:
def move_command(data):
    pass    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start scanop.py")
rospy.spin()
#blinker.stop()
