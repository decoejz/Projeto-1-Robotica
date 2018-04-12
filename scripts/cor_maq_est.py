#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
import encontra_objetos #Para procurar os objetos
import le_scan_sonny #Para verificar se o robo esta em perigo

from turtlebot3_msgs.msg import Sound


bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
#####################
#Identificação do objeto 1
media = []
centro = []
area = 0.0
#####################
#Identificação do objeto 2
objeto2 = False
#####################

tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.3E9
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados


def roda_todo_frame(imagem):
	print("frame")
	global cv_image

	global media
	global centro
	global area

	global objeto2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs

	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		
		media, centro, area = encontra_objetos.identifica_objeto_1(cv_image)
		objeto2 = encontra_objetos.identifica_objeto_2(cv_image)
		
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	

## Classes - estados

#Máquina que procura por objetos.
class Girando(smach.State):
    def __init__(self):
    	#alinhou1 = referente ao objeto1
        smach.State.__init__(self, outcomes=['perigo','alinhou1','enxergou2', 'girando'])

    def execute(self, userdata):
		global velocidade_saida
		global perigo_laser
		global emitir_som #######Usar aqui apenas para fazer teste de som! Ativar linha 118.

		if perigo_laser:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'perigo'
		
		elif area!=0: ##Verificar esse encontro direitinho!!!
			if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
				velocidade_saida.publish(vel)
				return 'girando'
			elif math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x): #if ou elif?? o do prof era if!
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
				velocidade_saida.publish(vel)
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhou1'

		elif objeto2:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'enxergou2'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))
			velocidade_saida.publish(vel)
			# emitir_som.publish(0)
			return 'girando'

#Máquina que reage quando o objeto 1 é encontrado
class Reage1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'centralizado', 'perigo'])

    def execute(self, userdata):
		global velocidade_saida
		global perigo_laser

		if perigo_laser:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'perigo'

		else:
			if media is None:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhando'

			if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhando'
			
			if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhando'
			
			else:
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'centralizado'

#Máquina que reage quando o objeto 2 é encontrado
class Reage2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['procurando', 'centralizado','perigo'])

    def execute(self, userdata):
		global velocidade_saida
		global perigo_laser
		global emitir_som #Ativar linha 173 quando estiver tudo funcionando sem o som apenas

		if perigo_laser:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'perigo'

		else:
			if objeto2:
				# emitir_som.publish(1)
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.7))
				velocidade_saida.publish(vel)
				return 'centralizado'

			else:##Ver de emitir sons
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'procurando'

#Máquina que reage quando o robo se encontra em perigo (alguma coisa muito próximo dele)
class Parar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seguro','perigo'])

    def execute(self, userdata):
		global velocidade_saida
		global perigo_laser
		global emitir_som #Ativar linha 197 quando estiver tudo funcionando sem o som apenas

		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)

		if perigo_laser:
			# emitir_som.publish(2)
			return 'perigo'
		else:
			return 'seguro'

# main
def main():
	global velocidade_saida
	global emitir_som
	global perigo_laser
	global buffer
	
	rospy.init_node('cor_maq_est')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	perigo_laser = rospy.Subscriber("/scan", LaserScan, le_scan_sonny.scaneou)

	emitir_som = rospy.Publisher("/sound", Sound)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    #smach.StateMachine.add('LONGE', Longe(), 
	    #                       transitions={'ainda_longe':'ANDANDO', 
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(), 
	    #                       transitions={'ainda_longe':'LONGE'})
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou1':'REACAO1','enxergou2':'REACAO2','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO1', Reage1(),
	                            transitions={'centralizado': 'REACAO1',
	                            'alinhando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('REACAO2', Reage2(),
	                            transitions={'centralizado': 'REACAO2',
	                            'procurando':'GIRANDO','perigo':'PERIGOSO'})
	    
	    smach.StateMachine.add('PERIGOSO', Parar(),
	                            transitions={'perigo': 'PERIGOSO',
	                            'seguro':'GIRANDO'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()