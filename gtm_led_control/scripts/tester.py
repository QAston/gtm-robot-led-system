#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from gtm_animation_msgs.msg import OneshotAnimationInfo, LoopAnimationInfo, LoopAnimation

rospy.init_node('led_control_tester')

battery_publisher = rospy.Publisher('battery_level', Int32, latch = True, queue_size = 1)

confirm_publisher = rospy.Publisher('oneshot_anim', OneshotAnimationInfo, queue_size = 1)

loop_publisher = rospy.Publisher('loop_animation', LoopAnimation, queue_size = 1)

loop_stop_publisher = rospy.Publisher('loop_animation_stop', LoopAnimationInfo, queue_size = 1)

rate = rospy.Rate(1)

battery_level = 100

while not rospy.is_shutdown():
  battery_publisher.publish(battery_level)
  
  if (battery_level % 10) == 0:
    confirm_publisher.publish(OneshotAnimationInfo(OneshotAnimationInfo.CONFIRM))
    
  if (battery_level % 10) == 0:
    if (battery_level % 20) == 0:
      msg = LoopAnimation()
      msg.animation = LoopAnimationInfo(LoopAnimationInfo.WAITING)
      msg.loop_duration = rospy.Duration(15)
      loop_publisher.publish(msg)
    else:
      loop_stop_publisher.publish(LoopAnimationInfo(LoopAnimationInfo.WAITING))
  
  battery_level -= 1
  if battery_level < 0:
    battery_level = 100
    

  
  rate.sleep()