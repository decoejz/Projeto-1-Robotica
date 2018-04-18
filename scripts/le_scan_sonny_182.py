#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


achou_perigo = False
direita_perigo = False
esquerda_perigo = False
frente_perigo = False
tras_perigo = False

def scaneou(dado):
	global achou_perigo

	global achou_perigo
	global direita_perigo
	global esquerda_perigo
	global frente_perigo
	global tras_perigo

	achou_perigo = False
	direita_perigo = False
	esquerda_perigo = False
	frente_perigo = False
	tras_perigo = False

	distancia_segura = 0.25

	for i in range(0,359,1):
		valor = dado.ranges[i]
		if valor < dado.range_max and valor > dado.range_min:
			if valor < distancia_segura:
				if (i<=45 and i>=0) or (i<=359 and i>315):
					frente_perigo = True
				
				if i<=135 and i>45:
					esquerda_perigo = True
				
				if i<=225 and i>135:
					tras_perigo = True

				if i<=315 and i>225:
					direita_perigo=True

				achou_perigo = True








