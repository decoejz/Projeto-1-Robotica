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
import findobjs as po
import numpy as np
# import le_scan as sc

bridge = CvBridge()

cv_image = None
objeto_1 = False
objeto_2 = False
perigo = False

atraso = 1.5

check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados
	
def contorno_quadrado():
	distancia_risco = 5#Checar qual a unidade(m, cm, mm....)
				
	#Testando se o robo nao esta em perigo.
	perigo = sc.laser_scan(distancia_risco)
	if perigo:
		vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
		return (vel)
	
	else:
		tempo_lado = 1#Ver quanto o robo precisa dormir para desenhar um lado
		velocidade_desenho = -0.5#Velocidade que ele andará em um lado do quadrado.

		vel = Twist(Vector3(velocidade_desenho,0,0), Vector3(0,0,0))#Velocidade para andar para frente
		velocidade_saida.publish(vel)#Aplicando a velocidade no robo
		rospy.sleep(tempo_lado)#Colocar o robo para dormir, para assim poder andar o espaço desejado.######VERIFICAR SE É ESSE O CÓDIGO DE DORMIR###
		
		###############################################################
		####Ver a necessidade de zerar a velocidade nesse ponto!!!!####
		###############################################################

		perigo = sc.laser_scan(distancia_risco)
		if perigo:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			return (vel)

		else:
			tempo_rotacao = 1#Checar quanto tempo precisa para ele virar apenas 90 graus

			vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))#Girando o robo em 90 graus. Ver qual a velocidade ideal para isso.
			velocidade_saida.publish(vel)
			rospy.sleep(tempo_rotacao)#Colocar o robo para dormir para poder girar os 90 graus.

			vel = Twist(Vector3(velocidade_desenho,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(tempo_lado)

			###############################################################
			####Ver a necessidade de zerar a velocidade nesse ponto!!!!####
			###############################################################

			perigo = sc.laser_scan(distancia_risco)
			if perigo:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
				return (vel)

			else:
				tempo_rotacao = 1#Checar quanto tempo precisa para ele virar apenas 90 graus

				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))#Girando o robo em 90 graus. Ver qual a velocidade ideal para isso.
				velocidade_saida.publish(vel)
				rospy.sleep(tempo_rotacao)#Colocar o robo para dormir para poder girar os 90 graus.

				vel = Twist(Vector3(velocidade_desenho,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)
				rospy.sleep(tempo_lado)

				###############################################################
				####Ver a necessidade de zerar a velocidade nesse ponto!!!!####
				###############################################################

				perigo = sc.laser_scan(distancia_risco)
				if perigo:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					return (vel)

				else:
					tempo_rotacao = 1#Checar quanto tempo precisa para ele virar apenas 90 graus

					vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))#Girando o robo em 90 graus. Ver qual a velocidade ideal para isso.
					velocidade_saida.publish(vel)
					rospy.sleep(tempo_rotacao)#Colocar o robo para dormir para poder girar os 90 graus.

					vel = Twist(Vector3(velocidade_desenho,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(tempo_lado)

					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					return (vel)


def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global objeto_1
	global objeto_2
	global perigo

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		#perigo = scaner()
		objeto_1 = po.findobj1(cv_image)
		#objeto_2 = po.findobj1(imagem)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":

	rospy.init_node("cor")
	# Para usar a Raspberry Pi
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)

			if perigo:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			elif objeto_1:
				vel = Twist(Vector3(-0.5,0,0), Vector3(0,0,0))

			elif objeto_2:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0.56))#contorno_quadrado()

			else:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))


			velocidade_saida.publish(vel)
			print(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


