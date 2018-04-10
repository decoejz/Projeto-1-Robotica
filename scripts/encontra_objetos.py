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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
import matplotlib.pyplot as plt
from math import pi

def identifica_objeto_1(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
	# vermelho puro (H=0) estão entre H=-8 e H=8. 
	# Precisamos dividir o inRange em duas partes para fazer a detecção 
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([0, 50, 50])
	cor_maior = np.array([8, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([172, 50, 50])
	cor_maior = np.array([180, 255, 255])
	segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
	# que um quadrado 7x7. É muito útil para juntar vários 
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno = None
	maior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media = maior_contorno.mean(axis=0)
	    media = media.astype(np.int32)
	    cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
	    media = (0, 0)

	# Representa a area e o centro do maior contorno no frame
	font = cv2.FONT_HERSHEY_COMPLEX_SMALL
	cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro = (frame.shape[0]//2, frame.shape[1]//2)

	return media, centro, maior_contorno_area

def identifica_objeto_2(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray,(1,1),0)
	v2 = np.median(blur)
	lower = int(max(0, (1.0 - 0.0001) * v2))
	upper = int(min(255, (1.0 + 0.0001) * v2))
	bordas = cv2.Canny(blur, lower, upper)    
	cv2.imwrite("test.jpg",frame)
	img1 = cv2.imread('pano2.jpg',0)          # Imagem a procurar
	img2 = cv2.imread("test.jpg",0)
	findCircle = False
	findFox =False
	circles = []
	bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
	circles = None
	circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=5,param2=150,minRadius=1,maxRadius=60)
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			findCircle = True
			cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
			cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
	MIN_MATCH_COUNT = 10
	if findCircle == True:
		sift = cv2.xfeatures2d.SIFT_create()
		kp1, des1 = sift.detectAndCompute(img1,None)
		kp2, des2 = sift.detectAndCompute(img2,None)
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(des1,des2,k=2)
		good = []
		for m,n in matches:
			if m.distance < 0.7*n.distance:
				good.append(m)
		if len(good)>MIN_MATCH_COUNT:
			findFox = True
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			matchesMask = mask.ravel().tolist()
			h,w = img1.shape
			pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			dst = cv2.perspectiveTransform(pts,M)
			img2b = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)
	sift = cv2.xfeatures2d.SIFT_create()
	kpts = sift.detect(frame)
	x = [k.pt[0] for k in kpts]
	y = [k.pt[1] for k in kpts]
	s = [(k.size/2)**2 * pi for k in kpts]
	if findFox == True:
		return(True)
	else:
		return(False)