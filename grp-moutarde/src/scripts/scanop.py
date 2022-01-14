#!/usr/bin/python3

from http import server
from math import sqrt
import rospy
import numpy as np
import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import PointCloud, LaserScan, Image, CameraInfo
from cv_bridge import CvBridge
from kobuki_msgs.msg import Led
import tf
import image_geometry
from std_srvs.srv import SetBool

# chemin = rospy.__path__  #+ "src/scripts/cascade.xml"
# print(chemin)

class Node:

    # constructor
    def __init__(self):
        self.img = np.array([[200, 33, 213]], np.uint8)
        self.bridge = CvBridge()
        self.classifier_debouts = cv2.CascadeClassifier("/home/grp-moutarde/Bureau/train_cascade/cascade_so.xml")
        #self.classifier_couches = cv2.CascadeClassifier("/home/grp-moutarde/Bureau/train_cascade/cascade3_c27.xml")
        self.cam_info = CameraInfo()
        self.img = 0
        self.depth = 0
        self.nb_detections = 0
        self.count_bouteille = 0
        self.img_recue = False
        self.depth_recue = False

        # Publishers
        self.publisher_led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        self.publisher_image = rospy.Publisher('/image_detection', Image, queue_size=10)
        self.publisher_image_depth = rospy.Publisher('/image_detection_depth', Image, queue_size=10)
        self.publisher_bouteilles = rospy.Publisher('/bottle', Marker, queue_size=10)

        # Listeners
        self.listener_img = rospy.Subscriber("/camera/color/image_raw", Image, self.perception_img)
        self.listener_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.perception_depth)
        self.listener_cam_info = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.perception_caminfo)

        # TF listener
        self.tf_listener = tf.TransformListener()

    def perception_img(self, data):
        if not self.img_recue:
            self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.img_recue = True
        self.actualiser(data.header.stamp)
            

    def perception_depth(self, data):
        if not self.depth_recue:
            self.depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.depth_recue = True
        self.actualiser(data.header.stamp)
            
    
    def actualiser(self, stamp):
        if self.img_recue and self.depth_recue:
            gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
            objets_debouts = self.classifier_debouts.detectMultiScale(gray, 1.2, minNeighbors=3)
            points = []
            for (x, y, w, h) in objets_debouts:
                cv2.rectangle(self.img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.rectangle(self.depth, (x, y), (x+w, y+h), (255, 255, 255), 2)
                dist = self.trouver_distance(x, y, w, h)
                point = self.calculer_point_central(x, y, w, h, dist)
                points.append(self.create_point_in_map_frame(point, stamp))
                #print(x, y, w, h, dist, point)
                #self.publier_bouteille(point, stamp)
            self.update_led(len(objets_debouts))
            Bottle.update(points, stamp)
        
            image_detection = self.bridge.cv2_to_imgmsg(self.img, encoding="rgb8")
            image_detection_depth = self.bridge.cv2_to_imgmsg(self.depth, encoding="16UC1")
            self.publisher_image.publish(image_detection)
            self.publisher_image_depth.publish(image_detection_depth)
            #cv2.imshow("img", self.img)
            #cv2.waitKey(33)
            self.img_recue = False
            self.depth_recue = False

    def perception_caminfo(self, data):
        self.cam_info = data

    def trouver_distance(self, x, y, w, h):
        centre_depth = self.depth[y+h//4:y+3*h//4, x+w//4:x+3*w//4] # récupérer centre du rectangle de détection
        dist = np.median(centre_depth) # prendre la médiane de ce rectangle permet de ne pas être affecté par les valeurs extremes comme l'arrière plan
        return dist / 1000 # convertir les mm en m

    def calculer_point_central(self, x, y, w, h, dist):
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(self.cam_info)
        ray = np.array(cam_model.projectPixelTo3dRay((x+w//2, y+h//2))) # transformer les coordonnées du pixel central en un point dans le repère de la caméra (à une distance de 1)
        point = ray * (dist + 0.022) # on multiplie donc par la distance mesurée par la caméra à laquelle on ajoute environ la moitié du diamètre de la bouteille
        return point

    def update_led(self, nb):
        if self.nb_detections != nb:
            self.nb_detections = nb
            msg = Led()
            msg.value = self.nb_detections
            if msg.value > 3: msg.value = 3
            self.publisher_led.publish(msg)
    
    # Converts tuple in camera frame to PointStamped in map frame
    def create_point_in_map_frame(self, point_in_camera_frame, stamp):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x, msg.point.y, msg.point.z = point_in_camera_frame
        return self.tf_listener.transformPoint("map", msg)

    def publier_bouteille(self, point, stamp):
        msg = Marker()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.header.stamp = stamp
        msg.ns = "Bouteille"
        msg.id = self.count_bouteille
        msg.type = 1
        msg.action = 0
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = point
        msg.pose.orientation.w = 1
        msg.scale.x, msg.scale.y, msg.scale.z = [0.1, 0.1, 0.1]
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = [0, 255, 0, 255]
        msg.lifetime.secs, msg.lifetime.nsecs = [2, 0]
        self.publisher_bouteilles.publish(msg)
        self.count_bouteille += 1


class Bottle:
    # Static variables
    bottles = []
    id_counter = 0

    # Parameters
    required_detections = 10 # Number of detections required for a bottle to be listed
    max_alive_time = 20 # Max number of frames the bottle is kept alive for without being detected before it is listed
    detection_distance = 0.1 # distance under which the bottles are considered the same

    # Bottle publisher
    publisher_bottle = rospy.Publisher('/bottle', Marker, queue_size=10)

    def __init__(self, point):
        Bottle.id_counter += 1
        self.id = Bottle.id_counter
        self.point = point # PointStamped in map frame
        self.detection_counter = 0 # Number of times the bottle has been detected
        self.kill_countdown = 0 # Number of frames the bottle has been alive for
        self.listed = False # Stays False until the bottle has been detected enough times
    
    def modify(self, point, stamp):
        self.point = point
        if not self.listed:
            self.detection_counter += 1
            self.kill_countdown = 0
            print(self.id, "nb detection", self.detection_counter)
            if self.detection_counter >= Bottle.required_detections:
                self.listed = True
                print(self.id, "listed")
        if self.listed:
            self.publish(stamp)

    def update(points, stamp):
        for p in points:
            already_exists = False
            for b in Bottle.bottles:
                if b.distance_to(p) < Bottle.detection_distance:
                    b.modify(p, stamp)
                    print(b.id, "updated")
                    already_exists = True
            if not already_exists:
                new_bottle = Bottle(p)
                Bottle.bottles.append(new_bottle) # Creation of a new bottle if it does not already exists
                print(new_bottle.id, "created")
        
        for b in Bottle.bottles:
            if not b.listed:
                b.kill_countdown += 1
                if b.kill_countdown >= Bottle.max_alive_time:
                    print(b.id, "killed")
                    Bottle.bottles.remove(b)
        
        print("====================================")

    # Calculate the distance between the bottle and a PointStamped
    def distance_to(self, point):
        return sqrt((self.point.point.x - point.point.x)**2 + (self.point.point.y - point.point.y)**2 + (self.point.point.z - point.point.z)**2)

    def publish(self, stamp):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp
        msg.ns = "Bouteille"
        msg.id = self.id
        msg.type = 1
        msg.action = 0
        msg.pose.position = self.point.point
        msg.pose.orientation.w = 1
        msg.scale.x, msg.scale.y, msg.scale.z = [0.1, 0.1, 0.1]
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = [0, 255, 0, 255]
        #msg.lifetime.secs, msg.lifetime.nsecs = [2, 0]
        Bottle.publisher_bottle.publish(msg)

# call the move_command at a regular frequency:
#rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

class Service:
    def __init__(self):
        rospy.Service("Print_Bottle", SetBool, self.bottle_srv)
    
    def bottle_srv(self,req):
        answer = ""
        for bo in Bottle.bottles:
            if bo.listed:
                answer += "\n" + "Bouteille : " + str(bo.id) + ", x : " + str(bo.point.point.x) + ", y : " + str(bo.point.point.y) + ", z : " + str(bo.point.point.z)
        if answer == "":
            answer = "Pas de Bouteille !"
        return (True, "Liste des bouteilles : " + answer)

rospy.init_node('scanop', anonymous=True)
node = Node()
service = Service()
print("Start scanop.py")
rospy.spin()
