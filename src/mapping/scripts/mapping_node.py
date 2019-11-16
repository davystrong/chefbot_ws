#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry

#0(preto) = sabemos que está ocupado (1)
#255(branco) = sabemos que está livre (0)
#100(cinza) = não sabemos (-1)
img = np.ones((200,200), np.float32) * 100
img_now = np.ones((200,200), np.float32) * 100
hasImage = False
LaserScanGlobal = {}
angle_Yaw = 0
x_a_moved = 0
y_a_moved = 0

pub = rospy.Publisher('/traj_output_teste', Image, queue_size=10)
pub2 = rospy.Publisher('/vision_now_matrix', Image, queue_size=10)

def Publish_Image():
    global img
    br = CvBridge()
    msg = br.cv2_to_imgmsg(img)
    pub.publish(msg)

def Publish_Image_Now(img_now):
    br = CvBridge()
    msg = br.cv2_to_imgmsg(img_now)
    pub2.publish(msg)

def CalculateNewRotationMatrix(angleRunned):
    return [[np.cos(angleRunned), np.sin(angleRunned)],[-np.sin(angleRunned),np.cos(angleRunned)]]

def movementCallback(Odometer):
    global x_a_moved
    global y_a_moved
    global angle_Yaw
    x_a_moved = int(Odometer.pose.pose.position.x)
    y_a_moved = int(Odometer.pose.pose.position.y)
    #print (Odometer.pose.pose.orientation)
    quaternion = (
        Odometer.pose.pose.orientation.x,
        Odometer.pose.pose.orientation.y,
        Odometer.pose.pose.orientation.z,
        Odometer.pose.pose.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    #print((roll,pitch,yaw))
    angle_Yaw = yaw

def kinectCallback(LaserScanLocal):
    global LaserScanGlobal
    global hasImage
    global angle_Yaw
    angle = 0
    global x_a_moved
    global y_a_moved
    x_a = 100 + x_a_moved
    y_a = 100 + y_a_moved
    radius = 2
    LaserScanGlobal = LaserScanLocal
    hasImage = True
    img_now = np.ones((200,200), np.float32) * 100
    #marca como livre a área que o robo ocupa
    img_now = cv2.circle(img_now, (x_a, y_a), radius, 255, -1)
    img     = cv2.circle(img, (x_a, y_a), radius, 255, -1)
    rotationMatrix = CalculateNewRotationMatrix(angle_Yaw)
    #print(angle_Yaw)
    angle = LaserScanLocal.angle_min
    for i in LaserScanLocal.ranges:
        angle = LaserScanLocal.angle_increment + angle
        if not math.isnan(i):
            #neste caso foi encontrado um obejto
            #calcula a distancia, nos eixos x e y, entre o objeto encontrado e o centro de massa do robo
            y_dist = i * np.sin(angle)
            x_dist = i * np.cos(angle)
            #print(angle)
            #rotaciona a distancia a partir da posição atual do robo
            x_dist_rotated = x_dist * rotationMatrix[0][0] + y_dist * rotationMatrix[0][1]
            y_dist_rotated = x_dist * rotationMatrix[1][0] + y_dist * rotationMatrix[1][1]
            #transforma a distancia calculada em pixels
            x_pix = int(x_dist_rotated * 10 + x_a)
            y_pix = int(y_dist_rotated * 10 + y_a)
            #traça uma linha branca indicando que tudo até o objeto encontrado não está ocupado
            cv2.line(img,(x_a,y_a),(x_pix , y_pix),255,1)
            cv2.line(img_now,(x_a,y_a),(x_pix , y_pix),255,1)

            cv2.line(img,(x_pix , y_pix),(x_pix , y_pix),0,1)
            cv2.line(img_now,(x_pix , y_pix),(x_pix , y_pix),0,1)
            #pinta o píxel encontrado de preto indicando que o mesmo está ocupado

    x_dist_rotated = 5 * rotationMatrix[0][0]
    y_dist_rotated = 5 * rotationMatrix[1][0]
    x_pix = int(x_dist_rotated * 10 + x_a)
    y_pix = int(y_dist_rotated * 10 + y_a)
    cv2.line(img_now,(x_a,y_a),(x_pix , y_pix), 180, 1)
    Publish_Image_Now(img_now)


def map():
    while not rospy.core.is_shutdown():
        Publish_Image()



if __name__ == '__main__':
    rospy.init_node('mapping_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, kinectCallback)
    rospy.Subscriber('/odom', Odometry, movementCallback)
    map()

    pass
