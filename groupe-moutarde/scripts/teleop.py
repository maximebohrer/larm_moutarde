#!/usr/bin/python3

import math, rospy, random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

obstacles=[]

commandPublisher = rospy.Publisher(
    '/cmd_vel',
    Twist, queue_size=10
)

def perception(data):
    global obstacles
    obstacles = data.ranges
    #angle_min = data.angle_min
    #angle_max = data.angle_max
    #angle_inc = data.angle_increment
    move_command(0)

commandListener = rospy.Subscriber("/scan", LaserScan , perception)

# Publish velocity commandes:
def move_command(data):
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    rand = random.randint(0,30)
    if min(obstacles) < 0.3:
        cmd.angular.z = 0.5
    elif min(obstacles[:100]) < 0.5:
        cmd.linear.x = 0.3
        cmd.angular.z = 0.5
    else:
        cmd.linear.x = 0.3
    commandPublisher.publish(cmd)

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start teleop.py")
rospy.spin()