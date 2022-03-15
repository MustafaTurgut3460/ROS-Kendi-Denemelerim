#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from find_object_2d.msg import ObjectsStamped

class NesneTanima():
   def __init__(self):
      rospy.init_node("nesne_tanima")
      rospy.Subscriber("objectsStamped", ObjectsStamped, self.nesneTani)
      rospy.spin()

   def nesneTani(self, mesaj):
      self.nesne_id = mesaj.objects.data[0]
      print(self.nesne_id)

NesneTanima()