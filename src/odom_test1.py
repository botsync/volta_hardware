#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist

def mover():
  global pub
  twist = Twist()
  twist.linear.x = 0.1
  twist.linear.y = 0
  twist.linear.z = 0
  twist.angular.x = 0
  twist.angular.y = 0
  twist.angular.z = 0
  pub.publish(twist)

  time.sleep(40)

  twist = Twist()
  twist.linear.x = 0
  twist.linear.y = 0
  twist.linear.z = 0
  twist.angular.x = 0
  twist.angular.y = 0
  twist.angular.z = 0
  pub.publish(twist)

  del pub

if __name__ == '__main__':
  pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
  rospy.init_node('mover', anonymous=True)
  time.sleep(1.0)
  print("Ready")
  try:
    mover()
    print ("Done")
  except KeyboardInterrupt:
    pass

