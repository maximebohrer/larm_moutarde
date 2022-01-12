#!/usr/bin/python3

import math
import rospy
import random
import numpy as np
import cv2
import kobuki_led_controller
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import image_geometry



# Initialize ROS::node
rospy.init_node('training', anonymous=True)
# chemin = rospy.__path__  #+ "src/scripts/cascade.xml"
# print(chemin)
img = np.array([[200, 33, 213]], np.uint8)
bridge = CvBridge()
object_cascade = cv2.CascadeClassifier("/home/grp-moutarde/catkin_ws/src/larm_moutarde/grp-moutarde/src/scripts/cascade1_deb.xml")
cam_info = CameraInfo()


#clignote = False
# blinker = kobuki_led_controller.LedBlinker()
# blinker.start()

# def blink(data):
#     blinker.set_on_error()
#     rospy.sleep(2)
#     blinker.set_on_ok
#     rospy.sleep(2)
#     blinker.set_on_off

depth = 0

c=0
def perception_img(data):
    # global c
    # c+=1
    # print(c)
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    object = object_cascade.detectMultiScale(gray, 1.2, minNeighbors=3)
    for (x, y, w, h) in object:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
        # rospy.Timer(rospy.Duration(2),blink,oneshot=True)
        print(calculer_point_central(x,y,w,h))
    #cv2.imshow("img", img)
    #cv2.waitKey(33)


depth = 0


def perception_depth(data):
    global depth
    depth = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    #depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("depth", depth)
    #cv2.waitKey(33)

def perception_caminfo(data):
    global cam_info
    cam_info = data


commandListener_img = rospy.Subscriber("/camera/color/image_raw", Image, perception_img)
commandListener_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, perception_depth)
commandListener_depth = rospy.Subscriber("/camera/color/camera_info", CameraInfo, perception_caminfo)


def trouver_distance(x, y, w, h):
    #return depth[x+w//2, y+h//2]
    centre_depth = depth[x+w//4:x+3*w//4, y+h//4:y+3*h//4] # récupérer centre du rectangle de détection
    dist = np.median(centre_depth) # prendre la médiane de ce rectangle permet de ne pas être affecté par les valeurs extremes comme l'arrière plan
    #print(dist)
    return dist

def calculer_point_central(x, y, w, h):
    cam_model = image_geometry.PinholeCameraModel()
    cam_model.fromCameraInfo(cam_info)
    ray = np.array(cam_model.projectPixelTo3dRay((x+w//2, y+h//2))) # transformer les coordonnées du pixel central en un point dans le repère de la caméra (à une distance de 1)
    point = ray * (trouver_distance(x, y, w, h) + 22) # on multiplie donc par la distance mesurée par la caméra à laquelle on ajoute environ la moitié du diamètre de la bouteille
    return point

# Publish velocity commandes:


def move_command(data):
    pass


# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start scanop.py")
rospy.spin()
# blinker.stop()
