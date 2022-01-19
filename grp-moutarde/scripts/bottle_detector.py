#!/usr/bin/python3

import rospy, rospkg, cv2, numpy as np, tf, image_geometry, message_filters
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from kobuki_msgs.msg import Led, Sound
from std_srvs.srv import Trigger
from math import sqrt, inf


class Node:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('grp-moutarde')
        self.bridge = CvBridge()
        self.classifier_standing_bottles = cv2.CascadeClassifier(self.pkg_path + "/scripts/cascade1_deb.xml")
        self.nb_detections = 0
        self.working = False

        # Publishers
        self.publisher_led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        self.publisher_image = rospy.Publisher('/image_detection', Image, queue_size=10)

        # Listeners
        self.camera_listener = message_filters.TimeSynchronizer([
            message_filters.Subscriber("/camera/color/image_raw", Image),
            message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image),
            message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
            ], 10)
        self.camera_listener.registerCallback(self.callback)

        # TF listener
        self.tf_listener = tf.TransformListener()
    
    # This functions gets called each time data is received from the camera. The TimeSynchronizer takes care of using 3 messages which have the same time stamp.
    def callback(self, color, depth, camera_info):
        if not self.working:
            self.working = True
            color_img = self.bridge.imgmsg_to_cv2(color, desired_encoding='passthrough')
            depth_img = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')

            # Detection in the frame and conversion to 3D coordinates
            gray = cv2.cvtColor(color_img, cv2.COLOR_RGB2GRAY)
            standing_bottles = self.classifier_standing_bottles.detectMultiScale(gray, 1.1, minNeighbors=3)
            points = []
            for (x, y, w, h) in standing_bottles:
                dist = self.find_distance(depth_img, x, y, w, h)
                h_times_dist = h * dist # The appearant height is inversely proportional to the distance, so h*dist is a constant. Comparing it to its expected value, which turns out to be between 140 and 180, gives us a way to filter out false alarms.
                if h_times_dist >= 140 and h_times_dist <= 180:
                    cv2.rectangle(color_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    point = self.find_center_point(camera_info, x, y, w, h, dist)
                    points.append(self.create_point_in_map_frame(point, color.header.stamp))
            self.update_led(len(points))

            # Create and update the position of the bottles
            Bottle.update(points, color.header.stamp)
        
            image_detection = self.bridge.cv2_to_imgmsg(color_img, encoding="rgb8")
            self.publisher_image.publish(image_detection)
            self.working = False

    # Calculate the distance between the camera and the rectangle using the depth image
    def find_distance(self, depth_img, x, y, w, h):
        center_depth = depth_img[y+h//3:y+2*h//3, x+w//3:x+2*w//3]      # get a small rectangle in the middle of the detected bottle
        dist = np.median(center_depth)                                  # take the median of this small rectangle to avoid beeing affected by extreme values like the background or parasites
        return dist / 1000                                              # convert millimeters to meters

    # Calculates the 3D coordinates of the center of the rectangle in the camera frame
    def find_center_point(self, camera_info, x, y, w, h, dist):
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(camera_info)
        ray = np.array(cam_model.projectPixelTo3dRay((x+w//2, y+h//2))) # convert the pixel coordinates of the center point to 3D coordinates in the camera frame (at a distance of 1)
        point = ray * (dist + 0.022)                                    # we then multiply by the distance that we calculated to which we add about half of the bottle's diameter.
        return point
    
    # Convert tuple in camera frame to PointStamped in map frame
    def create_point_in_map_frame(self, point_in_camera_frame, stamp):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.point.x, msg.point.y, msg.point.z = point_in_camera_frame
        return self.tf_listener.transformPoint("map", msg)

    # Light up the robot LED if bottles are detected
    def update_led(self, nb):
        if self.nb_detections != nb:
            self.nb_detections = nb
            msg = Led()
            msg.value = self.nb_detections
            if msg.value > 3: msg.value = 3
            self.publisher_led.publish(msg)
    

class Bottle:
    # Static variables
    bottles = []
    id_counter = 0

    # Parameters
    required_detections = 4             # Number of detections required for a bottle to be listed
    max_alive_time = 4                  # Max number of frames the bottle is kept alive for without being detected before it is listed
    detection_distance = 0.35           # distance under which the bottles are considered the same
    publish_updates = True              # Each time a bottle is detected, its position is adjusted. This variable determines if those adjustments should be published. If false, the marker will only be sent once. If true, the marker will be updated (re-published withe the SAME ID) each time the bottle is detected.

    # Publishers
    publisher_bottle = rospy.Publisher('/bottle', Marker, queue_size=10)
    publisher_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)

    # Creation of a new bottle
    def __init__(self, point):
        Bottle.id_counter += 1
        self.id = Bottle.id_counter
        self.point = point              # PointStamped in map frame
        self.detection_counter = 0      # Number of times the bottle has been detected
        self.kill_countdown = 0         # Number of frames the bottle has been alive for
        self.listed = False             # Stays False until the bottle has been detected enough times. Before that the bottle will not be published
    
    # Modification of an existing bottle
    def modify(self, point, stamp):
        x = (4 * self.point.point.x + point.point.x) / 5    # To smooth out the position of the bottle, we take the average
        y = (4 * self.point.point.y + point.point.y) / 5    # of the five last values.
        z = (4 * self.point.point.z + point.point.z) / 5
        self.point = point
        self.point.point.x, self.point.point.y, self.point.point.z = x, y, z
        if not self.listed:
            self.detection_counter += 1
            self.kill_countdown = 0
            if self.detection_counter >= Bottle.required_detections:
                self.listed = True
                self.publish(stamp)
                Bottle.beep()
        elif Bottle.publish_updates:
            self.publish(stamp)

    # Static function which runs each frame. The points variable contains all the PointStamped detected in the current frame.
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
                b.modify(nearest_point, stamp)          # This point is considered to correspond to an existing bottle. The bottle is adjusted with the coordinates of the new point.
                points.remove(nearest_point)            # Do not compare this point again to the next bottles. That way if multiple points are detected in one frame, they will all necessarily be treated as different bottles, even if they are close.

        for p in points:
            new_bottle = Bottle(p)
            Bottle.bottles.append(new_bottle)           # The remaining points, which do not correspond to any existing bottle, are added as new bottles.
        
        # Kill all the bottles that have not been listed and not detected for too long. This filters out false alarms
        for b in Bottle.bottles:
            if not b.listed:
                b.kill_countdown += 1
                if b.kill_countdown >= Bottle.max_alive_time:
                    Bottle.bottles.remove(b)

    # Calculate the distance between the bottle and a PointStamped
    def distance_to(self, point):
        return sqrt((self.point.point.x - point.point.x)**2 + (self.point.point.y - point.point.y)**2 + (self.point.point.z - point.point.z)**2)

    # Publish the bottle as a marker
    def publish(self, stamp):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp
        msg.ns = "bottle"
        msg.id = self.id
        msg.type = 1
        msg.action = 0
        msg.pose.position = self.point.point
        msg.pose.orientation.w = 1
        msg.scale.x, msg.scale.y, msg.scale.z = [0.1, 0.1, 0.1]
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = [0, 255, 0, 255]
        #msg.lifetime.secs, msg.lifetime.nsecs = [2, 0]
        Bottle.publisher_bottle.publish(msg)
    
    # Static function that returns the list of PointStamped correponding to all the listed bottles
    def get_bottles():
        bottles = []
        for b in Bottle.bottles:
            if b.listed:
                bottles.append(b.point)
        return bottles
    
    # Sound
    def beep():
        msg = Sound()
        msg.value = msg.ON
        Bottle.publisher_sound.publish(msg)
    
    def publish_obstacles():
        stamp = rospy.get_rostime()
        


class Service:
    def __init__(self):
        rospy.Service("print_bottles", Trigger, self.bottle_srv)
    
    def bottle_srv(self, req):
        bottles = Bottle.get_bottles()
        answer = str(len(bottles)) + " bottle(s) "
        for b in bottles:
            answer += "[" + str(round(b.point.x, 3)) + " " + str(round(b.point.y, 3)) + " " + str(round(b.point.z, 3)) + "] "
        return (True, answer)


rospy.init_node('bottle_detector', anonymous=True)
node = Node()
service = Service()
timer = rospy.Timer(rospy.Duration(0.1), Bottle.publish_obstacles, oneshot=False)
print("Start bottle_detector.py")
rospy.spin()