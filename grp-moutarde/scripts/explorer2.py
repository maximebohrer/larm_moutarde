#!/usr/bin/python3

import rospy, rospkg, cv2, numpy as np, tf, image_geometry, message_filters
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from kobuki_msgs.msg import Led, Sound
from std_srvs.srv import Trigger
from math import sqrt, inf
import time



class Node:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('grp-moutarde')
        self.bridge = CvBridge()
        self.classifier_standing_bottles = cv2.CascadeClassifier(self.pkg_path + "/src/scripts/cascade1_deb.xml")
        self.nb_detections = 0
        self.global_map = None

        # Publishers
        self.publisher_led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        self.publisher_image = rospy.Publisher('/image_detection', Image, queue_size=10)

        # Listeners
        self.listener_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # TF listener
        self.tf_listener = tf.TransformListener()

        self.sent=False
    
    def map_callback(self, data):
        t1 = time.perf_counter()
        img = np.resize(data.data, (data.info.height, data.info.width))
        t2 = time.perf_counter()
        for y in range(4000):
            for x in range(4000):
                img[y,x]=0
        t3 = time.perf_counter()
        print(t2 - t1, t3 - t2, data.info.origin.position.x, data.info.origin.position.y, data.info.origin.position.z)
    


rospy.init_node('explorer', anonymous=True)
node = Node()
print("Start explorer2.py")
rospy.spin()
