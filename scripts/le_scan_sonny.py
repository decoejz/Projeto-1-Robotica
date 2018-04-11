#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	global perigo_laser
	perigo_laser = False
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	# print(np.array(dado.ranges).round(decimals=2))
	distancia_segura = 0.45
	for i in range(-45,46,1):
		if i <=0:
			if dado.ranges[i] < dado.range_max and dado.ranges[i] > dado.range_min:
				valor = dado.ranges[i]
				print(valor)
				if valor < distancia_segura:
					velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
					velocidade_saida.publish(velocidade)
					print("Objeto não identificado na esquerda!!")
					return True	
				else:
					velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
					velocidade_saida.publish(velocidade)
					print("Vamo que vamo!!")
			else:
				velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(velocidade)
				perigo_laser = False
				print("Vamo que vamo cair fora!!")
		else:
			if dado.ranges[i] < dado.range_max and dado.ranges[i] > dado.range_min:
				valor = dado.ranges[i]
				print(valor)
				print("Sai da frente!!!")
				if valor < distancia_segura:
					velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
					velocidade_saida.publish(velocidade)
					print("Objeto não identificado na direita!!")
					return True	
				else:
					velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
					velocidade_saida.publish(velocidade)
					print("Vamo que vamo!!")
			else:
				velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(velocidade)
				perigo_laser = False
				print("Vamo que vamo cair fora!!")
	return False

	

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	while not rospy.is_shutdown():
		# velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		# velocidade_saida.publish(velocidade)
		rospy.sleep(2)
