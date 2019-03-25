#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
frontal=0 #definindo distancia inicial zero para poder trabalha la nos condicionais


def scaneou(dado):
	global frontal
	frontal = dado.ranges[0]
	

if __name__ == "__main__":
	rospy.init_node("roda_exemplo")
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	try:
		while not rospy.is_shutdown():
			print("parede a : {0}m ".format(round(frontal,2)))

			if frontal <= 1:
				vel = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(2.0)

			elif frontal >= 1.02:
				vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(2.0)

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")
