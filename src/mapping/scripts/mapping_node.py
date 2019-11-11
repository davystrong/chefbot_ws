#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

#0(preto) = sabemos que está ocupado (1)
#255(branco) = sabemos que está livre (0)
#100(cinza) = não sabemos (-1)
img = np.zeros((200,200))
hasImage = False
LaserScanGlobal = {}

pub = rospy.Publisher('/traj_output', Image, queue_size=10)

def Publish_Image():
    global img
    br = CvBridge()
    msg = br.cv2_to_imgmsg(img)
    pub.publish(msg)

def CalculateNewRotationMatrix(angleRunned):
    return [[np.cos(angleRunned), np.sin(angleRunned)],[-np.sin(angleRunned),np.cos(angleRunned)]]

def kinectCallback(LaserScan):
    global LaserScanGlobal
    global hasImage
    LaserScanGlobal = LaserScan
    hasImage = True
    cv2.imshow('mapa', img)

def map():
    global maxAngle
    global LaserScanGlobal
    global hasImage
    global img
    x_a = 100
    y_a = 100
    rotationMatrix = CalculateNewRotationMatrix(map.angleRunned)
    while not rospy.core.is_shutdown():
        if hasImage:
            for i in LaserScanGlobal.ranges:
                try:
                    #neste caso foi encontrado um obejto
                    #calcula a distancia, nos eixos x e y, entre o objeto encontrado e o centro de massa do robo
                    map.angle = LaserScanGlobal.angle_increment + map.angle
                    y_dist = i * np.sin(map.angle)
                    x_dist = i * np.cos(map.angle)

                    #rotaciona a distancia a partir da posição atual do robo
                    x_dist_rotated = x_dist * rotationMatrix[0][0] + y_dist * rotationMatrix[0][1]
                    y_dist_rotated = x_dist * rotationMatrix[1][0] + y_dist * rotationMatrix[1][1]

                    #transforma a distancia calculada em pixels
                    x_pix = x_dist_rotated * 0.5 + x_a
                    y_pix = y_dist_rotated * 0.5 + y_a
                    #traça uma linha branca indicando que tudo até o objeto encontrado não está ocupado
                    cv2.line(img,(x_a,y_a),(x_pix , y_pix),255,1)
                    #pinta o píxel encontrado de preto indicando que o mesmo está ocupado
                    img[x_pix, y_pix] = 0
                    map.angle = LaserScan.angle_increment + map.angle
                    Publish_Image()

                except:
                    #neste caso não foi encontrado nenhum objeto no campo de visão da câmera
                    #calcula a distancia, nos eixos x e y, entre o objeto encontrado e o centro de massa do robo
                    map.angle = LaserScanGlobal.angle_increment + map.angle
                    y_dist = LaserScanGlobal.range_max * np.sin(map.angle)
                    x_dist = LaserScanGlobal.range_max * np.cos(map.angle)

                    #rotaciona a distancia a partir da posição atual do robo
                    x_dist_rotated = x_dist * rotationMatrix[0][0] + y_dist * rotationMatrix[0][1]
                    y_dist_rotated = x_dist * rotationMatrix[1][0] + y_dist * rotationMatrix[1][1]

                    #transforma a distancia calculada em pixels
                    x_pix = int(x_dist_rotated * 0.5 + x_a)
                    y_pix = int(y_dist_rotated * 0.5 + y_a)
                    #traça uma linha branca indicando que tudo até o objeto encontrado não está ocupado
                    cv2.line(img,(x_a,y_a),(x_pix , y_pix),255,1)
                    Publish_Image()




            #rotate 90 degrees
            map.angleRunned += 90
            rotationMatrix = CalculateNewRotationMatrix(map.angleRunned)
            hasImage = False

map.angle = 0
map.angleRunned = 0

if __name__ == '__main__':
    rospy.init_node('mapping_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, kinectCallback)
    map()

    pass
