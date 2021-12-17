#!/usr/bin/python3

import math, rospy, random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud 
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

nuage = []
fluid = 0
savecmd = Twist()

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
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    mind = 3
    ming = 3
    global fluid
    global savecmd

    for i in nuage:
        if i.x > 0 and i.x < 0.3:
            if i.y < mind:
                mind = i.y
        if i.x < 0 and i.x > -0.3:
            if i.y < ming:
                ming = i.y

    if mind < 0.15 or ming < 0.15:
        cmd.angular.z = 2
        fluid = 0
    elif fluid != 0:
        cmd = savecmd
        fluid -= 1
    else:
        if ming < 3 and ming > mind:
            cmd.angular.z = 1/mind
            cmd.linear.x = 0.3 * mind
            if cmd.angular.z > 2:
                cmd.angular.z = 2

            if ming < 0.3:
                savecmd = cmd
                fluid = 10

        elif mind < 3 and ming < mind:
            cmd.angular.z = -1/ming
            cmd.linear.x = 0.3 * ming
            if cmd.angular.z < -2: 
                cmd.angular.z = -2
            
            if mind < 0.3:
                savecmd = cmd
                fluid = 10
            
        else:
            cmd.linear.x = 0.3 * mind
        
    commandPublisher.publish(cmd)

    cmd_rviz = TFMessage()
    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start teleop2.py")
rospy.spin()