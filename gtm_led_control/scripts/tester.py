#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


rospy.init_node('led_control_tester')

pub = rospy.Publisher('battery_level', Int32, latched = True)

rate = rospy.Rate(1)

battery_level = 100

while not rospy.is_shutdown():
  pub.publish(battery_level)
  battery_level -= 1
  if battery_level < 0:
    battery_level = 100
  rate.sleep()