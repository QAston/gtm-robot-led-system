#!/usr/bin/env python
""" led_control node
Topics subscribed to:
battery_level - int32 value 0-100 to display, latched
opc_info - string adress:port of opc server
"""


import rospy
from std_msgs.msg import Int32, String
import opc
import time
import random


def battery_level_changed(msg):
  rospy.loginfo("Got new battery level %s", msg.data)

client = None

def opc_server_changed(msg):
  global client
  if client is not None:
    rospy.loginfo("Opc server changed, killing the old opc connection...")
    client.disconnect()
  client = opc.Client(msg.data)
  if client.can_connect():
    rospy.loginfo("Connected to new opc server %s!", msg.data)
  else:
    rospy.logerr("Could not connect to new opc server %s!", msg.data)
  

rospy.init_node('led_control')
sub = rospy.Subscriber('battery_level', Int32, battery_level_changed)
sub = rospy.Subscriber('opc_info', String, opc_server_changed)

# Send pixels forever
while not rospy.is_shutdown():
  if client is not None:
    my_pixels = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    random.shuffle(my_pixels)
    if client.put_pixels(my_pixels, channel=0):
        pass
    else:
        rospy.logerr("Could not send pixels to opc server!")
    rospy.sleep(1.0)