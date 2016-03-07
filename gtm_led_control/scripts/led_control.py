#!/usr/bin/env python
""" led_control node
Topics subscribed to:
battery_level - int32 value 0-100 to display



"""


import rospy
from std_msgs.msg import Int32



def callback(msg):
  print msg.data

rospy.init_node('led_control')
sub = rospy.Subscriber('battery_level', Int32, callback)
rospy.spin()