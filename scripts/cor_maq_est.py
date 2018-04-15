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
media1 = []
centro1 = []
area1 = 0.0
#####################
#Identificação do objeto 2
# objeto2 = False
media2 = []
centro2 = []
area2 = 0.0
#####################

tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.1
area1_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area1 = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.2E9
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados


def roda_todo_frame(imagem):
	print("frame")
	global cv_image

	global media1
	global centro1
	global area1

	# global objeto2
	global media2
	global centro2
	global area2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs

	if delay > atraso and check_delay==True:
		# print("\t\tDELAY: ", delay/1.0E9)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		
		media1, centro1, area1 = encontra_objetos.identifica_objeto_1(cv_image)
		# objeto2 = encontra_objetos.identifica_objeto_2(cv_image)
		media2, centro2, area2 = encontra_objetos.identifica_objeto_2(cv_image)
		
		depois = time.clock()
		# print("Demorou: ", depois - antes)
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	

## Classes - estados

#Máquina que procura por objetos.
#Caso esteja em perigo ele para de procurar;
#Caso não esteja em perigo, verifica se o objeto 1 esta
#sendo capturado pela camera;
#Caso não esteja em perigo, nem tenho o objeto 1,
#ele procura o objeto 2;
#Caso contrário, ele continua girando até encontrar alguma coisa.
class Girando(smach.State):
    def __init__(self):
    	#alinhou1 = referente ao objeto1
        smach.State.__init__(self, outcomes=['perigo','alinhou1','enxergou2', 'girando'])

    def execute(self, userdata):
		global velocidade_saida

		if le_scan_sonny.achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			print('PERIGO')
			return 'perigo'
		
		elif len(media1) != 0 and len(centro1) != 0:#area1!=0: ##Verificar esse encontro direitinho!!!
			if  math.fabs(media1[0]) > math.fabs(centro1[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed)) #Talvez inverter o sentido da rotacao por conta de a camera estar de ponta cabeca
				velocidade_saida.publish(vel)
				print('Para direita')
				return 'girando'
			elif math.fabs(media1[0]) < math.fabs(centro1[0] - tolerancia_x): #if ou elif?? o do prof era if!
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed)) #Talvez inverter o sentido da rotacao por conta de a camera estar de ponta cabeca
				velocidade_saida.publish(vel)
				print('Para esquerda')
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Alinhou com 1')
				return 'alinhou1'

		# elif objeto2:
		# 	vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		# 	velocidade_saida.publish(vel)
		# 	print('Achou 2')
		# 	return 'enxergou2'
		elif len(media2) != 0 and len(centro2) != 0:#area1!=0: ##Verificar esse encontro direitinho!!!
			if  math.fabs(media2[0]) > math.fabs(centro2[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed)) #Talvez inverter o sentido da rotacao por conta de a camera estar de ponta cabeca
				velocidade_saida.publish(vel)
				print('Para direita')
				return 'girando'
			elif math.fabs(media2[0]) < math.fabs(centro2[0] - tolerancia_x): #if ou elif?? o do prof era if!
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed)) #Talvez inverter o sentido da rotacao por conta de a camera estar de ponta cabeca
				velocidade_saida.publish(vel)
				print('Para esquerda')
				return 'girando'
			else:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Alinhou com 1')
				return 'enxergou2'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			print('Girar e girar')
			return 'girando'

#Máquina que reage quando o objeto 1 é encontrado
#Caso o objeto 1 esteja na tela e o robo não esteja em perigo,
#Ele seguirá o objeto 1.
class Reage1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'centralizado', 'perigo'])

    def execute(self, userdata):
		global velocidade_saida

		if le_scan_sonny.achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			print('PERIGO')
			return 'perigo'

		else:
			if media1 is None:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Sem objeto 1')
				return 'alinhando'

			elif  math.fabs(media1[0]) > math.fabs(centro1[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Para direita')
				return 'alinhando'
			
			elif math.fabs(media1[0]) < math.fabs(centro1[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Para esquerda')
				return 'alinhando'
			
			else:
				vel = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('NO centro1 de 1')
				return 'centralizado'

#Máquina que reage quando o objeto 2 é encontrado
#Caso o objeto 2 esteja na tela e o objeto 1 não e o robo nao esteja em perigo,
#O robo girará e emitirá um som.
class Reage2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['procurando', 'centralizado','perigo'])

    def execute(self, userdata):
		global velocidade_saida
		# global emitir_som

		if le_scan_sonny.achou_perigo:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			# emitir_som.publish(0)
			print('PERIGO')
			return 'perigo'

		else:
			# if objeto2:
			# 	# emitir_som.publish(0)
			# 	vel = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
			# 	velocidade_saida.publish(vel)
			# 	print('Ta com o objeto 2')
			# 	return 'centralizado'

			# else:
			# 	vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			# 	velocidade_saida.publish(vel)
			# 	# emitir_som.publish(0)
			# 	print('Sem objeto 2')
			# 	return 'procurando'
			if media2 is None:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Sem objeto 2')
				return 'procurando'

			elif  math.fabs(media2[0]) > math.fabs(centro2[0] + tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Para direita')
				return 'procurando'
			
			elif math.fabs(media2[0]) < math.fabs(centro2[0] - tolerancia_x):
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('Para esquerda')
				return 'procurando'
			
			else:
				vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('NO centro2 de 2')
				return 'centralizado'

#Máquina que reage quando o robo se encontra em perigo (alguma coisa muito próximo dele).
#Caso ele se encontre em perigo, o robo parará de andar e emitirá um som avisando sobre 
#o perigo.
class Parar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seguro','perigo'])

    def execute(self, userdata):
		global velocidade_saida
		# global emitir_som

		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)

		if le_scan_sonny.achou_perigo:
			# emitir_som.publish(0)
			print('PERIGO')
			return 'perigo'
		else:
			# emitir_som.publish(0)#Tentar mandar null para ver se para
			print('Em SEGURANCA')
			return 'seguro'

# main
def main():
	global velocidade_saida
	global emitir_som
	global buffer
	
	rospy.init_node('cor_maq_est')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	#Define a velocidade quando chamada.
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	#Roda a função que verifica se o robo esta em perigo ou não.
	perigo_laser_objeto = rospy.Subscriber("/scan", LaserScan, le_scan_sonny.scaneou)

	#Define o som quando chamada.
	# emitir_som = rospy.Publisher("/sound", Sound)

	print("PRINT TESTE")

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
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


if __name__ == '__main__':
    main()