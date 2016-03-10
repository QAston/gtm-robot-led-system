#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Empty
from gtm_animation_msgs.msg import OneshotAnimationInfo, LoopAnimationAction, LoopAnimationGoal, LoopAnimationResult, LoopAnimationInfo
import actionlib

rospy.init_node('led_control_tester')

battery_publisher = rospy.Publisher('battery_level', Int32, latch = True, queue_size = 1)

confirm_publisher = rospy.Publisher('oneshot_anim', OneshotAnimationInfo, queue_size = 1)

client = actionlib.SimpleActionClient('loop_animation', LoopAnimationAction)
client.wait_for_server()

rate = rospy.Rate(1)

battery_level = 100

while not rospy.is_shutdown():
  battery_publisher.publish(battery_level)
  
  if (battery_level % 10) == 0:
    confirm_publisher.publish(OneshotAnimationInfo(OneshotAnimationInfo.CONFIRM))
    
  if (battery_level % 10) == 0:
    if (battery_level % 20) == 0:
      goal = LoopAnimationGoal()
      goal.animation = LoopAnimationInfo(LoopAnimationInfo.WAITING)
      client.send_goal(goal)
    else:
      rospy.loginfo("send goal cancel")
      client.cancel_all_goals()
  
  battery_level -= 1
  if battery_level < 0:
    battery_level = 100
    

  
  rate.sleep()