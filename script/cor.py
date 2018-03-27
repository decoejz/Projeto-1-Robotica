#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import findobjs as procura_obj

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	


if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	
	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
			achou_obj_1 = procura_obj.findobj1(recebedor)
			
			if achou_obj_1:
				vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
# 			if len(media) != 0 and len(centro) != 0:
# 				dif_x = media[0]-centro[0]
# 				dif_y = media[1]-centro[1]
# 				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
# 			# 		vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
# 			# 	else:
# 			# 		if dif_x > 0: # Vira a direita
# 			# 			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
# 			# 		else: # Vira a esquerda
# 			# 			vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
# 			# velocidade_saida.publish(vel)
# 			# rospy.sleep(0.01)
# ######################################################################################################
# 					#print("dif_x",dif_x)##########Saber qual a grandeza desse valor###########################
# ######################################################################################################
# 					#Fazer uma condição if, elif, else para diferentes distancias.
# 					#Quanto mais perto, menos velocidade
# 					#Quanto mais longe, mais velocidade
# 					#vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))##Codigo original do professor
# ######################################################################################################					
# 					if dif_x>20:
# 						vel = Twist(Vector3(-1,0,0), Vector3(0,0,0))
# 					elif dif_x>10:
# 						vel = Twist(Vector3(-0.7,0,0), Vector3(0,0,0))
# 					elif dif_x>5:
# 						vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))
# 					elif dif_x>2:
# 						vel = Twist(Vector3(-0.3,0,0), Vector3(0,0,0))
# 					else:
# 						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
# 				else:
# 					if dif_x > 0: # Vira a direita
# 						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
# 					else: # Vira a esquerda
# 						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(vel)
			print(vel)
			rospy.sleep(0.01)
######################################################################################################

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


