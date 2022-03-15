#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from basit_uygulamalar.srv import CemberHareket

rospy.wait_for_service("cember_servis")

try:
   yaricap = float(input("Yaricap Giriniz: "))
   servis = rospy.ServiceProxy("cember_servis", CemberHareket)
   servis(yaricap)
except rospy.ServiceException:
   print("Servis ile alakalı bir hata olustu!")