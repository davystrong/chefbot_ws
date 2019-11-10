# This Python file uses the following encoding: utf-8
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan

#0(preto) = sabemos que está ocupado (1)
#255(branco) = sabemos que está livre (0)
#100(cinza) = não sabemos (-1)

img = np.zeros((200,200), datatype = int) # global variable
hasImage = false # global variable
LaserScanGlobal # global variable

def CalculateNewRotationMatrix(angleRunned):
    return [[np.cos(angleRunned), np.sin(angleRunned)],[-np.sin(angleRunned),np.cos(angleRunned)]]

def kinectCallback(LaserScan):
    for i in len(LaserScan.range):
        global LaserScanGlobal = LaserScan
        global hasImage = true

if__name__ == "__main__":
    rospy.init_node('mapping_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, kinectCallback)
    angleRunned = 0
    rotationMatrix = CalculateNewRotationMatrix(angleRunned)
    while not rospy.core.is_shutdown():
        if hasImage:
            for i in len(LaserScan.range):
                try:
                    #neste caso foi encontrado um obejto
                    #calcula a distancia, nos eixos x e y, entre o objeto encontrado e o centro de massa do robo
                    y_dist = LaserScan.range[i]np.sin(LaserScan.angle_min + LaserScan.angle_incrementi)
                    x_dist = LaserScan.range[i]np.cos(LaserScan.angle_min + LaserScan.angle_increment*i)

                    #rotaciona a distancia a partir da posição atual do robo
                    x_dist_rotated = x_dist * rotationMatrix[0][0] + y_dist * rotationMatrix[0][1]
                    y_dist_rotated = x_dist * rotationMatrix[1][0] + y_dist * rotationMatrix[1][1]

                    #transforma a distancia calculada em pixels
                    x_pix = x_dist_rotated * res + x_a
                    y_pix = y_dist_rotated * res + y_a
                    #traça uma linha branca indicando que tudo até o objeto encontrado não está ocupado
                    cv2.line(img,(x_a,y_a),(x_pix , y_pix),255,1)
                    #pinta o píxel encontrado de preto indicando que o mesmo está ocupado
                    img(x_pix, y_pix) = 0
                except:
                    #neste caso não foi encontrado nenhum objeto no campo de visão da câmera
                    #calcula a distancia, nos eixos x e y, entre o objeto encontrado e o centro de massa do robo
                    y_dist = LaserScan.range_max * np.sin(LaserScan.angle_min + LaserScan.angle_increment*i)
                    x_dist = LaserScan.range_max * np.cos(LaserScan.angle_min + LaserScan.angle_increment*i)

                    #rotaciona a distancia a partir da posição atual do robo
                    x_dist_rotated = x_dist * rotationMatrix[0][0] + y_dist * rotationMatrix[0][1]
                    y_dist_rotated = x_dist * rotationMatrix[1][0] + y_dist * rotationMatrix[1][1]

                    #transforma a distancia calculada em pixels
                    x_dist_rotated = x_dist * res + x_a
                    y_dist_rotated = y_dist * res + y_a
                    #traça uma linha branca indicando que tudo até o objeto encontrado não está ocupado
                    cv2.line(img,(x_a,y_a),(x_pix , y_pix),255,1)

           #rotate 90 degrees
           angleRunned += 90
           rotationMatrix = CalculateNewRotationMatrix(angleRunned)
           global hasImage = false
    pass
