#!/usr/bin/python3

import math, rospy, random
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

plt.axis([0,1,0,10])

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

nuage = []

commandPublisher = rospy.Publisher(
    '/cmd_vel',
    Twist, queue_size=10
)

def perception(data):
    global nuage
    nuage.clear
    plt.clf()
    for i in range(len(data.ranges)):
        angle = data.angle_min + i * data.angle_increment
        x = math.sin(angle) * data.ranges[i]
        y = math.cos(angle) * data.ranges[i]
        if y > 0:
            nuage.append([x,y])
            plt.scatter(x, y)
    plt.pause(0.05)

commandListener = rospy.Subscriber("/scan", LaserScan, perception)

def peut_avancer():
    for i in nuage:
        if i[0] > -0.2 and i[0] < 0.2 and i[1] < 0.2:
            return False
    return True

# Publish velocity commandes:
def move_command(data):
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    if peut_avancer():
        cmd.linear.x = 0.1
    else:
        cmd.angular.z = 0.2
    commandPublisher.publish(cmd)

    cmd_rviz = TFMessage()
    

# call the move_command at a regular frequency:
rospy.Timer(rospy.Duration(0.1), move_command, oneshot=False)

# spin() enter the program in a infinite loop
print("Start teleop2.py")
rospy.spin()