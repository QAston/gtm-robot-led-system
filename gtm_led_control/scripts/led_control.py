#!/usr/bin/env python
""" led_control node
Topics subscribed to:
battery_level - int32 value 0-100 to display, latched
opc_info - string adress:port of opc server
oneshot_anim - OneshotAnimationInfo - when message received starts a oneshot animation with given id
loop_anim - LoopAnimation - starts/continues a loop animation for given duration
loop_anim_stop - LoopAnimationInfo - stops a loop animation with given id
"""

import rospy
from std_msgs.msg import Int32, String
from gtm_animation_msgs.msg import OneshotAnimationInfo, LoopAnimationInfo, LoopAnimation
import opc
import time
import random
import math

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

#leds
#currently there're 3 8-led strings, 1 12-led circle
first_stripe_idx = 0
stripe_len = 8
second_stripe_idx = 8
third_stripe_idx = 64
circle_idx = 128
circle_len = 12
total_len = 140
max_brightness = 150
brightness_coeff = 150.0/255

#animations
#priority - integer assumed constant, class member
#done - true when animation finished
#animate - draws animation frame on given pixels
class IdleAnimation:
  priority = 0
  
  def __init__(self):
    self.battery_level = 0
    self.noise_clock = 0.01

  def done(self):
    return False
    
  def animate(self, pixels, clock, clockdiff):
    """
    Draws default animations
    """
    #background
    for i in range(total_len):
      pixels[i] = (0,0,0)

    self.draw_battery(pixels)
    self.draw_noise(pixels, clockdiff)
  
  def draw_noise(self, pixels, clockdiff):
    """
    Random noise on other stripes when nothing's happening
    """
    self.noise_clock -= clockdiff
    if (self.noise_clock < 0):
      self.noise_clock = random.gauss(5, 10) * 0.001 + 0.005
      
      bar_start = first_stripe_idx
      bar_length = 2 * second_stripe_idx
      indexes = [random.randrange(bar_start, bar_start + bar_length) for i in range(4)]
      colors = [(225,247,213), (255,189,189), (201,201,255), (254,255,163)]
      for i in range(4):
        pixels[bar_start + indexes[i]] = ((i * random.randint(1, 4) * 1.0 / 20)   for i in colors[i])
    

  def draw_battery(self, pixels):
    """
    Draws a green/red bar on the third stripe
    """
    bar_start = third_stripe_idx
    bar_len = stripe_len
    green_part = (self.battery_level * 1.0 / 100) * stripe_len
    
    green_pixels = int(math.floor(green_part))
    mixed_pixels = 1
    red_pixels = int(stripe_len - green_pixels - mixed_pixels)
    mixed_coeff = green_part - green_pixels
    mixed_color = (int((1-mixed_coeff) * max_brightness), int(mixed_coeff * max_brightness), 0)
    for i in range(bar_start, bar_start + green_pixels):
      pixels[i] = (0, max_brightness, 0)
    pixels[bar_start + green_pixels] = mixed_color
    red_begin = bar_start + green_pixels + 1
    for i in range(red_begin, red_begin + red_pixels):
      pixels[i] = (max_brightness, 0, 0)
    
  def battery_level_changed(self, level):
    rospy.loginfo("Got new battery level %s", level)
    self.battery_level = level

#loop animation
# should have set_duration method taking rospy.Duration
class WaitingAnimation:
  bar_start = circle_idx
  bar_len = circle_len
  priority = 3
  
  def __init__(self):
    self.start_clock = None
    self.loop_anim_duration = rospy.Duration(0)
  
  def done(self):
    return self.loop_anim_duration <= rospy.Duration(0)
    
  def set_duration(self, loop_duration):
    self.loop_anim_duration = loop_duration

  def animate(self, pixels, clock, clockdiff):
    if self.start_clock is None:
      self.start_clock = clock
      
    self.loop_anim_duration = self.loop_anim_duration - rospy.Duration(clockdiff)

    duration = 1
    head_pos = math.fmod(clock - self.start_clock, duration) * WaitingAnimation.bar_len / duration

    for i in range(WaitingAnimation.bar_start, WaitingAnimation.bar_start + WaitingAnimation.bar_len, 1):
      distance = math.fmod(head_pos - (i - WaitingAnimation.bar_start) + WaitingAnimation.bar_len, WaitingAnimation.bar_len)
      pixels[i] = self.pos_to_color(distance)
      
  def pos_to_color(self, distance):
    scaled_dist = (WaitingAnimation.bar_len - distance) / WaitingAnimation.bar_len
    if distance < 1:
      scale =  scaled_dist * brightness_coeff
      return (255 * scale, 153 * scale, 0 * scale)
    if distance < 1.5:
      scale = scaled_dist * brightness_coeff * 0.4
      return (255 * scale, 204 * scale, 0 * scale)
    if distance < 3:
      scale = scaled_dist * brightness_coeff *2 - 0.6
      return (133 * scale, 163 * scale, 216 * scale)
    
    return (0 , 0, 0)

#oneshot animation
class ConfirmAnimation:
  bar_start = first_stripe_idx
  bar_len = stripe_len * 2
  priority = 9
  
  def __init__(self):
    self.start_clock = None
    self.is_done = False

  def done(self):
    return self.is_done

  def animate(self, pixels, clock, clockdiff):
    if self.start_clock is None:
      self.start_clock = clock

    duration = 1
    if (clock - self.start_clock) > duration:
      self.is_done = True
      return
    current = math.fmod(clock - self.start_clock, duration) * ConfirmAnimation.bar_len * 2 / duration
    head_pos = abs(ConfirmAnimation.bar_len -  current)
    if current <= ConfirmAnimation.bar_len:
      step = 1
      start = ConfirmAnimation.bar_start
      end = ConfirmAnimation.bar_start + ConfirmAnimation.bar_len
    else:
      step = -1
      start = ConfirmAnimation.bar_start + ConfirmAnimation.bar_len - 1
      end = ConfirmAnimation.bar_start -1

    for i in range(start, end, step):
      distance = ((i - ConfirmAnimation.bar_start) - head_pos) * step
      pixels[i] = self.pos_to_color(distance)
      
  def pos_to_color(self, distance):
    scaled_dist = (ConfirmAnimation.bar_len - distance) / ConfirmAnimation.bar_len
    if (distance < 0):
      return (0,0,0)
    if distance < 1:
      scale =  scaled_dist * brightness_coeff
      return (255 * scale, 153 * scale, 0 * scale)
    if distance < 2.5:
      scale = scaled_dist * brightness_coeff * 0.4
      return (255 * scale, 204 * scale, 0 * scale)
    if distance < 10:
      scale = scaled_dist * brightness_coeff *1.3 - 0.4
      return (133 * scale, 163 * scale, 216 * scale)
    
    return (0 , 0, 0)

animations = [IdleAnimation()]
oneshot_animation_classes = [ConfirmAnimation]
loop_animation_classes = [WaitingAnimation]


def add_animation(anim, remove_callback = None):
  animations.insert(anim.__class__.priority, anim)
  
def remove_animation(anim):
  if len(animations) > anim.__class__.priority:
    animations[anim.__class__.priority] = None

def animate():
  rate = rospy.Rate(20)
  pixels = [(0,0,0) for x in range(total_len)]
  last_clock = time.time()
  clock = last_clock
  while not rospy.is_shutdown():
    last_clock = clock
    clock = time.time()
    clock_diff = clock - last_clock
    if client is not None:
      for anim in animations:
        if (anim is None):
          continue
        anim.animate(pixels, clock, clock_diff)
        if (anim.done()):
          remove_animation(anim)
      if client.put_pixels(pixels, channel=0):
        pass
      else:
        rospy.logerr("Could not send pixels to opc server!")
    rate.sleep()

def oneshot_req_received(msg):
  rospy.loginfo("Requested starting oneshot animation %s!", msg.animation_id)
  anim_class = list_get(oneshot_animation_classes, msg.animation_id)
  if anim_class is None:
    rospy.logerr("Invalid oneshot animation id %s, skipped.", msg.animation_id)
    return
  add_animation(anim_class())
  
def list_get(l, i):
  if i < len(l) and i >= 0:
    return l[i]
  return None
  
def loop_continue_received(msg):
  rospy.loginfo("Requested starting/continuing loop animation %s!", msg.animation.animation_id)
  anim_class = list_get(loop_animation_classes, msg.animation.animation_id)
  if anim_class is None:
    rospy.logerr("Invalid loop animation id %s, skipped.", msg.animation.animation_id)
    return
  
  existing_anim = list_get(animations, anim_class.priority)
  if existing_anim is not None:
    existing_anim.set_duration(msg.loop_duration)
    rospy.loginfo("Animation already active, continuing...")
  else:
    existing_anim = anim_class()
    existing_anim.set_duration(msg.loop_duration)
    add_animation(existing_anim)
    rospy.loginfo("Started loop animation.")
    
  
  
def loop_cancel_received(msg):
  rospy.loginfo("Requested canceling loop animation %s!", msg.animation_id)
  anim_class = list_get(loop_animation_classes, msg.animation_id)
  if anim_class is None:
    rospy.logerr("Invalid loop animation id %s, skipped.", msg.animation_id)
    return

  existing_anim = list_get(animations, anim_class.priority)
  if existing_anim is not None:
    remove_animation(existing_anim)
    rospy.loginfo("Loop animation id %s cancelled.", msg.animation_id)

#node code:
rospy.init_node('led_control')
battery_sub = rospy.Subscriber('battery_level', Int32, lambda msg: animations[0].battery_level_changed(msg.data))
confirm_anim_sub = rospy.Subscriber('oneshot_anim', OneshotAnimationInfo, oneshot_req_received)
opc_info_sub = rospy.Subscriber('opc_info', String, opc_server_changed)
loop_sub = rospy.Subscriber('loop_animation', LoopAnimation, loop_continue_received)
loop_stop_sub = rospy.Subscriber('loop_animation_stop', LoopAnimationInfo, loop_cancel_received)

animate()

