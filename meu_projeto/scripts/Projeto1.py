#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from math import pi

batida=0
#frontal=0
v=0.1
w =(pi/2)

def bateu(dado):
	global batida
	batida = dado.data

# def scaneou(dado):
# 	global frontal
# 	frontal = dado.ranges[0]

# def hit()

# 	vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
# 	pub.publish(vel)
# 	rospy.sleep(1.0)
	
# 	vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
# 	pub.publish(vel)
# 	rospy.sleep(2.0)
	
# 	vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
# 	pub.publish(vel)
# 	rospy.sleep(2.0)
	
# 	vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
# 	pub.publish(vel)
# 	rospy.sleep(2.0)

#batida=0


if __name__ == "__main__":
	rospy.init_node("roda_exemplo")
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
	recebe_Bumper = rospy.Subscriber("/bumper", UInt8, bateu)
	#recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	
	try:
		while not rospy.is_shutdown():
			#print("parede a : {0}m ".format(round(frontal,2)))
			print("oi")
			#vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
			# print("to andano")
			# pub.publish(vel)
			# rospy.sleep(2.0)
			#batida=0

			if batida == 0:
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(2.0)
				print("andando")

			elif batida == 3:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)

				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(2.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
				pub.publish(vel)
				rospy.sleep(1.0)

				batida=0

			elif batida == 4:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)


				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(2.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
				pub.publish(vel)
				rospy.sleep(1.0)

				batida=0

			elif batida == 1:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)

				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
				pub.publish(vel)
				rospy.sleep(1.0)

				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(4.0)

				batida=0

			elif batida == 2:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)

				vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
				pub.publish(vel)
				rospy.sleep(1.0)
				
				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(4.0)
				
				vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
				pub.publish(vel)
				rospy.sleep(1.0)

				vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(4.0)

				batida=0

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")



