#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LazerVerisi():
   def __init__(self):
      rospy.init_node("lazer_dugumu")
      self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
      self.hiz_mesaji = Twist()
      rospy.Subscriber("scan", LaserScan, self.lazerCallback)
      rospy.spin()

   def lazerCallback(self, mesaj):
      sol_on = list(mesaj.ranges[0:9])
      sag_on = list(mesaj.ranges[350:359])
      on = sol_on + sag_on
      sol = list(mesaj.ranges[80:100])
      sag = list(mesaj.ranges[260:280])
      arka = list(mesaj.ranges[170:190])

      min_on = min(on)
      min_sol = min(sol)
      min_sag = min(sag)
      min_arka = min(arka)

      print(min_on)

      if min_on < 1.0:
         self.hiz_mesaji.linear.x = 0.0
         self.pub.publish(self.hiz_mesaji)

         time.sleep(2)

         # sola don
         self.donus(1, 3)

         # duz git
         self.duzGit(1)

         # saga don
         self.donus(-1, 3)

         # duz git
         self.duzGit(2)

         # saga don
         self.donus(-1, 3)

         # duz git
         self.duzGit(1)

         print("Görev Tamamlandı!")

      else:
         self.hiz_mesaji.angular.z = 0.0
         self.hiz_mesaji.linear.x = 0.25
         self.pub.publish(self.hiz_mesaji)

   def duzGit(self, mesafe:int):
      self.hiz_mesaji.linear.x = 0.25
      t0 = rospy.Time.now().to_sec()
      yer_degistirme = 0

      while mesafe > yer_degistirme:
         self.pub.publish(self.hiz_mesaji)
         t1 = rospy.Time.now().to_sec()
         yer_degistirme = self.hiz_mesaji.linear.x * (t1-t0)

      self.hiz_mesaji.linear.x = 0.0
      self.pub.publish(self.hiz_mesaji)
      print(str(mesafe) + " birim duz gidildi")

   def donus(self, yon:int, sure):
      self.hiz_mesaji.angular.z = yon * 0.5
      t0 = rospy.Time.now().to_sec()

      while True:
         self.pub.publish(self.hiz_mesaji)
         t1 = rospy.Time.now().to_sec()
         if t1-t0 >= sure:
            print("Donus bitti")
            break

      self.hiz_mesaji.angular.z = 0.0
      self.pub.publish(self.hiz_mesaji)


lazer = LazerVerisi()

