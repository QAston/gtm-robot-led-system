#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Empty


rospy.init_node('led_control_tester')

battery_publisher = rospy.Publisher('battery_level', Int32, latch = True, queue_size = 1)

confirm_publisher = rospy.Publisher('oneshot_anim_confitm', Empty, queue_size = 1)

rate = rospy.Rate(1)

battery_level = 100

while not rospy.is_shutdown():
  battery_publisher.publish(battery_level)
  
  if (battery_level % 10) == 0:
    confirm_publisher.publish()
  
  battery_level -= 1
  if battery_level < 0:
    battery_level = 100
    

  
  rate.sleep()