#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("donus")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
hiz_mesaji = Twist()
rate = rospy.Rate(10)
hiz_mesaji.angular.z = 0.5
hiz_mesaji.linear.x = 0.25

try:

   while not rospy.is_shutdown():
      pub.publish(hiz_mesaji)
      rate.sleep()

except rospy.ROSInterruptException:
   print("Donus Bitti!")
   rospy.is_shutdown()