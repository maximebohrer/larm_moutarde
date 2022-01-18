#!/usr/bin/python3

import math, rospy, random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import LaserScan

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

v_a_max = 0.4
v_l_max = 0.3

tout_droit = 0
tourne_gauche = 1
tourne_droite = 2
mode = tout_droit

nuage = []
vitesse_courante = 0
acceleration = 0.03

commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
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
        if data.ranges[i] > 0.02 and data.ranges[i] < 10:
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
    global mode, vitesse_courante

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
    mindg = min(mind, ming)

    vitesse_a_atteindre = 0.7 * (mindg - 0.10)
    if vitesse_a_atteindre < 0: vitesse_a_atteindre = 0
    if vitesse_a_atteindre > v_l_max: vitesse_a_atteindre = v_l_max

    if vitesse_courante < vitesse_a_atteindre:
        vitesse_courante += acceleration
    else:
        vitesse_courante = vitesse_a_atteindre
    cmd.linear.x = vitesse_courante

    if mode == tout_droit and ming < 0.18:
        mode = tourne_droite
    
    elif mode == tout_droit and mind < 0.18:
        mode = tourne_gauche
    
    elif (mode != tout_droit and mindg > 0.19):
        mode = tout_droit
    
    if mode == tourne_gauche:
        cmd.angular.z = v_a_max
    elif mode == tourne_droite:
        cmd.angular.z = -v_a_max
        
    commandPublisher.publish(cmd)
    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start explorer.py")
rospy.spin()