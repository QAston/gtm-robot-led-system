#!/usr/bin/env python
""" opc_server node
Topics published
opc_info - adress:port to which opc client should connect, latched
"""
import os
import sys
import rospy
from std_msgs.msg import String
if os.name == 'posix' and sys.version_info[0] < 3:
  import subprocess32 as subprocess
else:
  import subprocess

rospy.init_node('opc_server')
pub = rospy.Publisher('opc_info', String, latch = True, queue_size=1)

while not rospy.is_shutdown():
  process = subprocess.Popen(['/opt/fadecandy/fcserver'], universal_newlines=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  rospy.sleep(2)
  if process.poll() is None:
    rospy.loginfo("Succesfully started fadecandy server.")
    pub.publish("127.0.0.1:7890")
  else:
    rospy.logfatal("Failed to start fadecandy server, errorcode: %s", process.returncode)
    rospy.signal_shutdown("Failed to start fadecandy server")
  while (not rospy.is_shutdown()) and (process.poll() is None):
    rospy.loginfo("Fadecandy server output: %s", process.stdout.readline())
  if not rospy.is_shutdown():
    rospy.logerr("Fadecandy server died, errorcode: %s, attempting to restart...", process.returncode)