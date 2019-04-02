#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule

print("ta rodando o certo")
bridge = CvBridge()

cv_image = None
mediaV = []
centroV = []

mediaA = []
centroA = []

atraso = 0.5E9 # 1 segundo e meio. Em nanossegundos

areaV = 0.0 # Variavel com a area do maior contorno
areaA = 0.0
metade=320
sigma=25

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	global cv_image
	global mediaA
	global mediaV
	global centroA
	global centroV
	global areaV
	global areaA

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		mediaV, centroV, areaV =  cormodule.identifica_cor_vermelho(cv_image)
		mediaA, centroA, areaA =  cormodule.identifica_cor_azul(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)

	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	#print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			
			if (len(mediaV) != 0 and len(centroV) != 0 and len(mediaA) !=0  and len(centroA) !=0):
				
			#if areaA != 0 or areaV != 0:
				#print("passei")
				if areaV > areaA:
					
					if mediaV[0] < metade + sigma:
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("direitaa vermelho")
					
					elif mediaV[0] > metade + sigma:
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("esquerdaa vermelho")
					
					else:
						vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("em frente pro vermelho!")
				
				elif areaV < areaA:
					
					if mediaA[0] < metade + sigma:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,-0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("direitaa foge do azul")
					
					elif mediaA[0] > metade + sigma:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("esquerdaa run do azul")
					
					else:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("corre do azul berg!")
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					print("ahead we go!")
			#print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
			#print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
			#vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			#print("aqui")
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


