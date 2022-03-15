#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# BASARİZ BİR DENEME OLDU BURADA BİRAKTİM

import numpy as np
import rospy
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time


class EngelDolas():
   def __init__(self):
       rospy.init_node("engel_dolas")
       self.bridge = CvBridge()
       rospy.Subscriber("camera/rgb/image_raw", Image)
       rospy.Subscriber("scan", LaserScan, self.lazerCallback)
       self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
       self.hiz_mesaji = Twist()
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

      print("Uzaklik: " + str(min_on))

      # kamera
      img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      alt_sari = np.array([20, 100, 100])
      ust_sari = np.array([40, 255, 255])
      maske = cv2.inRange(hsv, alt_sari, ust_sari)
      sonuc = cv2.bitwise_and(img, img, mask=maske)

      h,w,d = img.shape
      cv2.circle(img, (int(w/2), int(h/2)), 5, (0, 0, 255), -1)

      M = cv2.moments(maske)

      if min_on < 1.0:
         print("Engel Var!")
         time.sleep(3)
         self.hiz_mesaji.linear.x = 0.0
         self.hiz_mesaji.angular.z = 0.5

         t0 = rospy.Time.now().to_sec()
         while True:
            self.pub.publish(self.hiz_mesaji)
            t1 = rospy.Time.now().to_sec()

            if t1-t0 >= 2:
               break

         self.hiz_mesaji.angular.z = 0.0
         self.hiz_mesaji.linear.x = 0.0
         self.pub.publish(self.hiz_mesaji)

         #self.kameraCallback(mesaj)

      else:
         if M["m00"] > 0:
            print("Serit bulundu, takip ediliyor...")
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            cv2.circle(img, (cx,cy), 5, (255, 0, 0), -1)

            sapma = cx - w/2
            self.hiz_mesaji.linear.x = 0.2
            self.hiz_mesaji.angular.z = -sapma/100
            #print(self.hiz_mesaji.angular.z)
            self.pub.publish(self.hiz_mesaji)

         else:
            print("Serit bulunamadi")
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)

         cv2.imshow("Orijinal", img)
         #cv2.imshow("Maske", maske)
         #cv2.imshow("Sonuc", sonuc)

         cv2.waitKey(1)

   def kameraCallback(self, mesaj):
      img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      alt_sari = np.array([20, 100, 100])
      ust_sari = np.array([40, 255, 255])
      maske = cv2.inRange(hsv, alt_sari, ust_sari)
      sonuc = cv2.bitwise_and(img, img, mask=maske)

      h,w,d = img.shape
      cv2.circle(img, (int(w/2), int(h/2)), 5, (0, 0, 255), -1)

      M = cv2.moments(maske)
      if M["m00"] > 0:
         print("Serit bulundu, takip ediliyor...")
         cx = int(M["m10"]/M["m00"])
         cy = int(M["m01"]/M["m00"])
         cv2.circle(img, (cx,cy), 5, (255, 0, 0), -1)

         sapma = cx - w/2
         self.hiz_mesaji.linear.x = 0.2
         self.hiz_mesaji.angular.z = -sapma/100
         #print(self.hiz_mesaji.angular.z)
         self.pub.publish(self.hiz_mesaji)

      else:
         print("Serit bulunamadi")
         self.hiz_mesaji.linear.x = 0.0
         self.hiz_mesaji.angular.z = 0.0
         self.pub.publish(self.hiz_mesaji)

      cv2.imshow("Orijinal", img)
      #cv2.imshow("Maske", maske)
      #cv2.imshow("Sonuc", sonuc)

      cv2.waitKey(1)

EngelDolas()