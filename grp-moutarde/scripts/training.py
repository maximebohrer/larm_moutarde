#!/usr/bin/python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image

img=np.array([[0,0,0]], np.uint8)

rospy.init_node('training', anonymous=True)

c=0
numimg = 0

def perception(data):
    global img, c, numimg
    c+=1
    if c==20:
        numimg +=1
        img = np.array(list(data.data), np.uint8)
        img = np.resize(img, (data.height, data.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(numimg)+".jpg", img)
        print("images/"+str(numimg)+".jpg")
        c=0

commandListener = rospy.Subscriber("/camera/color/image_raw", Image, perception)

print("Start training.py")
rospy.spin()