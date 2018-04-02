#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	# print(np.array(dado.ranges).round(decimals=2))
	distancia_segura = 0.5
	valor_minimo = 0.6
	var=0
	for i in range(90):
		if dado.ranges[i-45] < dado.range_max and dado.ranges[i-45] > dado.range_min:
			valor = dado.ranges[i-45]
			print(valor)
	
			if valor < distancia_segura:
				var += 1
				if var > 5:
					velocidade = Twist(Vector3(-1, 0, 0), Vector3(0, 0, 1))
					velocidade_saida.publish(velocidade)
					return True
			else:
				velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(velocidade)
				return False

		else:
			velocidade = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(velocidade)
			return False

		print(np.array(dado.ranges).round(decimals=2))
		#print("Intensities")
		#print(np.array(dado.intensities).round(decimals=2))

	

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	while not rospy.is_shutdown():
		# velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		# velocidade_saida.publish(velocidade)
		rospy.sleep(2)
