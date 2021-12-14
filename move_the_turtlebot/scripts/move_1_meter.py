#!/usr/bin/python3

compteur = 0

import math, rospy
from geometry_msgs.msg import Twist

# Initialize ROS::node
rospy.init_node('move', anonymous=True)

commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)

# call the move_command at a regular frequency:
t = rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

# Publish velocity commandes:
def move_command(data):
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()
    cmd.linear.x= 0.1
    commandPublisher.publish(cmd)
    compteur += 1
    if compteur == 10:
        t.shutdown()
        



# spin() enter the program in a infinite loop
print("Start move.py")
print("test")
rospy.spin()