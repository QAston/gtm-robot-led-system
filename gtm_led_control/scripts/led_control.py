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

class IdleAnimation:
  def __init__(self):
    self.battery_level = 0
    self.noise_clock = 0.01
    
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

#calc target pixel
#calc other pixels relative to target pixel
class WaitingAnimation:
  bar_start = circle_idx
  bar_len = circle_len
  
  def __init__(self):
    self.start_clock = None

  def animate(self, pixels, clock, clockdiff):
    if self.start_clock is None:
      self.start_clock = clock

    duration = 0.02
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

class ConfirmAnimation:
  bar_start = first_stripe_idx
  bar_len = stripe_len * 2
  
  def __init__(self):
    self.start_clock = None

  def animate(self, pixels, clock, clockdiff):
    if self.start_clock is None:
      self.start_clock = clock

    duration = 0.05
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

animations = [IdleAnimation(), WaitingAnimation(), ConfirmAnimation()]

def animate():
  rate = rospy.Rate(20)
  pixels = [(0,0,0) for x in range(total_len)]
  last_clock = time.clock()
  clock = last_clock
  while not rospy.is_shutdown():
    last_clock = clock
    clock = time.clock()
    clock_diff = clock - last_clock
    if client is not None:
      for anim in animations:
        anim.animate(pixels, clock, clock_diff)
      if client.put_pixels(pixels, channel=0):
        pass
      else:
        rospy.logerr("Could not send pixels to opc server!")
    rate.sleep()

#node code:
rospy.init_node('led_control')
sub = rospy.Subscriber('battery_level', Int32, lambda msg: animations[0].battery_level_changed(msg.data))
sub = rospy.Subscriber('opc_info', String, opc_server_changed)
animate()

