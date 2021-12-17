#!/usr/bin/python3

import math, rospy, random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import LaserScan

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

v_a_max = 0.7
v_l_max = 0.5

tout_droit = 0
tourne_gauche = 1
tourne_droite = 2
mode = tout_droit

nuage = []

commandPublisher = rospy.Publisher(
    '/cmd_vel',
    Twist, queue_size=10
)

nuagePublisher = rospy.Publisher(
    '/nuage',
    PointCloud, queue_size=10
)

def perception(data):
    global nuage
    nuage = []
    nu = PointCloud()
    for i in range(len(data.ranges)):
        if data.ranges[i] > 0.05 and data.ranges[i] < 10:
            angle = data.angle_min + i * data.angle_increment
            p = Point()
            p.x = -math.sin(angle) * data.ranges[i]
            p.y = math.cos(angle) * data.ranges[i]
            if p.y > 0:
                nuage.append(p)
    nu.points = nuage
    nu.header.frame_id = "laser_sensor_link"
    nuagePublisher.publish(nu)



commandListener = rospy.Subscriber("/scan", LaserScan, perception)

# Publish velocity commandes:
def move_command(data):
    global mode

    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    mind = 3
    ming = 3
    
    for i in nuage:
        if i.x > 0 and i.x < 0.3:
            if i.y < mind:
                mind = i.y
        if i.x < 0 and i.x > -0.3:
            if i.y < ming:
                ming = i.y

    cmd.linear.x = 0.4 * (min(mind, ming) - 0.15)
    if cmd.linear.x < 0: cmd.linear.x = 0
    if cmd.linear.x > v_l_max: cmd.linear.x = v_l_max

    if mode == tourne_gauche and (mind < 0.4 or ming < 0.4):
        cmd.angular.z = 1/v_a_max
    
    elif mode == tourne_droite and (mind < 0.4 or ming < 0.4):
        cmd.angular.z = -1/v_a_max

    elif ming > mind:
        mode = tourne_gauche
        cmd.angular.z = 1/mind
        if cmd.angular.z > v_a_max: cmd.angular.z = v_a_max

    elif ming < mind:
        mode = tourne_droite
        cmd.angular.z = -1/ming
        if cmd.angular.z < -v_a_max: cmd.angular.z = -v_a_max

    else:
        mode = tout_droit
        
    commandPublisher.publish(cmd)
    print(ming, mind)
    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start teleop2.py")
rospy.spin()