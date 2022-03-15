#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def hareket():
   rospy.init_node("duz_git")
   pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
   hiz_mesaji = Twist()

   hiz_mesaji.linear.x = 0.5
   #hiz_mesaji.linear.y = 0.5
   mesafe = 5
   yer_degistirme = 0
   t0 = rospy.Time.now().to_sec()

   while mesafe > yer_degistirme:
      pub.publish(hiz_mesaji)
      t1 = rospy.Time.now().to_sec()
      yer_degistirme = hiz_mesaji.linear.x * (t1-t0)

   hiz_mesaji.linear.x = 0.0
   pub.publish(hiz_mesaji)
   rospy.loginfo("Hedefe Varildi!")

hareket()
