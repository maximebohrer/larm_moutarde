#!/usr/bin/python3

import rospy, rospkg, cv2, numpy as np, tf, image_geometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from kobuki_msgs.msg import Led
from std_srvs.srv import Trigger
from math import sqrt, inf


class Node:
    # constructor
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('grp-moutarde')
        self.bridge = CvBridge()
        self.classifier_debouts = cv2.CascadeClassifier(self.pkg_path + "/src/scripts/cascade1_deb.xml")
        #self.classifier_couches = cv2.CascadeClassifier("/home/grp-moutarde/Bureau/train_cascade/cascade3_c27.xml")
        self.cam_info = CameraInfo()
        self.nb_detections = 0
        self.img_recue = False
        self.depth_recue = False

        # Publishers
        self.publisher_led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        self.publisher_image = rospy.Publisher('/image_detection', Image, queue_size=10)
        self.publisher_image_depth = rospy.Publisher('/image_detection_depth', Image, queue_size=10)

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
            objets_debouts = self.classifier_debouts.detectMultiScale(gray, 1.1, minNeighbors=3)
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
            #self.publisher_image.publish(image_detection)
            #self.publisher_image_depth.publish(image_detection_depth)
            #cv2.imshow("img", self.img)
            #cv2.waitKey(33)
            self.img_recue = False
            self.depth_recue = False

    def perception_caminfo(self, data):
        self.cam_info = data

    def trouver_distance(self, x, y, w, h):
        centre_depth = self.depth[y+h//4:y+3*h//4, x+w//4:x+3*w//4]     # récupérer centre du rectangle de détection
        dist = np.median(centre_depth)                                  # prendre la médiane de ce rectangle permet de ne pas être affecté par les valeurs extremes comme l'arrière plan
        return dist / 1000                                              # convertir les mm en m

    def calculer_point_central(self, x, y, w, h, dist):
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(self.cam_info)
        ray = np.array(cam_model.projectPixelTo3dRay((x+w//2, y+h//2))) # transformer les coordonnées du pixel central en un point dans le repère de la caméra (à une distance de 1)
        point = ray * (dist + 0.022)                                    # on multiplie donc par la distance mesurée par la caméra à laquelle on ajoute environ la moitié du diamètre de la bouteille
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
        return self.tf_listener.transformPoint("odom", msg)


class Bottle:
    # Static variables
    bottles = []
    id_counter = 0

    # Parameters
    required_detections = 4             # Number of detections required for a bottle to be listed
    max_alive_time = 5                  # Max number of frames the bottle is kept alive for without being detected before it is listed
    detection_distance = 0.3            # distance under which the bottles are considered the same

    # Bottle publisher
    publisher_bottle = rospy.Publisher('/bottle', Marker, queue_size=10)

    def __init__(self, point):
        Bottle.id_counter += 1
        self.id = Bottle.id_counter
        self.point = point              # PointStamped in map frame
        self.detection_counter = 0      # Number of times the bottle has been detected
        self.kill_countdown = 0         # Number of frames the bottle has been alive for
        self.listed = False             # Stays False until the bottle has been detected enough times. Before that the bottle will not be published
    
    def modify(self, point, stamp):
        x = (4 * self.point.point.x + point.point.x) / 5    # To smooth out the position of the bottle, we take the average
        y = (4 * self.point.point.y + point.point.y) / 5    # of the five last values.
        z = (4 * self.point.point.z + point.point.z) / 5
        self.point = point
        self.point.point.x, self.point.point.y, self.point.point.z = x, y, z
        if not self.listed:
            self.detection_counter += 1
            self.kill_countdown = 0
            print(self.id, "nb detection", self.detection_counter)
            if self.detection_counter >= Bottle.required_detections:
                self.listed = True
                print(self.id, "listed")
        if self.listed:
            self.publish(stamp)

    # Function which runs each frame. points contains all the PointStamped detected in the current frame.
    def update(points, stamp):
        for b in Bottle.bottles:
            nearest_point = None
            lowest_dist = inf
            for p in points:
                dist = b.distance_to(p)
                if dist < lowest_dist:
                    lowest_dist = dist
                    nearest_point = p
            if lowest_dist < Bottle.detection_distance:
                b.modify(nearest_point, stamp)             # This point is considered to correspond to an existing bottle. The bottle is adjusted with the coordinates of the new point.
                points.remove(nearest_point)        # Do not compare this point again to the next bottles. That way if multiple points are detected in one frame, they will all necessarily be treated as different bottles, even if they are close.
                print(b.id, "updated")

        for p in points:
            new_bottle = Bottle(p)
            Bottle.bottles.append(new_bottle)          # The remaining points, which do not correspond to any existing bottle, are added as new bottles.
            print(new_bottle.id, "created")
        
        # Kill all the bottles that have not been listed and not detected for too long. This filters out false alarms
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

    # Publish the bottle as a marker
    def publish(self, stamp):
        msg = Marker()
        msg.header.frame_id = "odom"
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
    
    # Returns the list of PointStamped correponding to all the listed bottles
    def get_bottles():
        bottles = []
        for b in Bottle.bottles:
            if b.listed:
                bottles.append(b.point)
        return bottles


class Service:
    def __init__(self):
        rospy.Service("print_bottles", Trigger, self.bottle_srv)
    
    def bottle_srv(self,req):
        bottles = Bottle.get_bottles()
        answer = str(len(bottles)) + " bottle(s) "
        for b in bottles:
            answer += "[" + str(round(b.point.x, 3)) + " " + str(round(b.point.y, 3)) + " " + str(round(b.point.z, 3)) + "] "
        return (True, answer)


rospy.init_node('scanop', anonymous=True)
node = Node()
service = Service()
print("Start scanop.py")
rospy.spin()
