#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import rospy

pubAng = rospy.Publisher('/wheels_angulo', Float32, queue_size=10)
pubVel = rospy.Publisher('/wheels_velocidade', Float32, queue_size=10)

def movementCallback(Odometer):
    global x_a_moved
    global y_a_moved
    global angle_Yaw
    global img
    quaternion = (
        Odometer.pose.pose.orientation.x,
        Odometer.pose.pose.orientation.y,
        Odometer.pose.pose.orientation.z,
        Odometer.pose.pose.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    mutex.acquire()
    angle_Yaw = yaw
    x_a_moved = Odometer.pose.pose.position.x
    y_a_moved = Odometer.pose.pose.position.y
    mutex.release()
    
def Gauss_Seidel(mapMatrix):
    grad = np.zeros(mapMatrix.shape)
    x = mapMatrix.shape[0]
    y = mapMatrix.shape[1]
    for i in range(1, x-1):
        for j in range(1, y-1):
            grad[i,j] = (-grad[i-1,j]+mapMatrix[i+1,j]-grad[i,j-1]+mapMatrix[i,j+1])/4.0
    grad = grad[1:x-1, 1:y-1]
    return grad

def mapping_callback(fullmap):
    grad = Gauss_Seidel(fullmap)
    grad = grad - grad.min()
    grad = grad/grad.max()
    grad[grad>0.99] = 1
    
    wh = np.where(grad == grad.max())
    
    D = 0
    X = 0
    Y = 0
    for i in range(0, wh[:][0].size):
        x = wh[0][i] - fullmap.shape[0]/2.0
        y = wh[1][i] - fullmap.shape[1]/2.0
        d = np.sqrt(x**2+y**2)
        if d>D:
            X = x
            Y = y
            D = d

    theta = math.atan2(Y,X)
    
    pubAng.publish(theta-angle_Yaw)
    pubVel.publish(D*0.1)

if __name__ == '__main__':
    rospy.init_node('exploration_node', anonymous=True)
    rospy.Subscriber('/traj_output_teste', Image, mapping_callback)
    rospy.Subscriber('/odom', Odometry, movementCallback)
    rospy.spin()

    pass
