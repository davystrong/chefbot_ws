# -*- coding: utf-8 -*-
"""
Spyder Editor

"""

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

realmap = np.zeros((200,200), np.float)

cv2.rectangle(realmap, (0,0), (49,49), 1, -1)
cv2.rectangle(realmap, (149,0), (189,199), 1, -1)
cv2.circle(realmap, (50,165), 25, 1,-1)

plt.imshow(realmap, cmap='Greys')

frame = np.ones_like(realmap)*(-1)
N=3
for i in range(0, 360*N-1):
        print(i)
        theta = i*math.pi/(N*180)
        s = 1
        px = np.int32(s*math.cos(theta)+100)
        py = np.int32(s*math.sin(theta)+100)
        while (px < 200 and py < 200 and px>=0 and py >=0):
            if (realmap[px,py] == 1):
                frame[px,py] = 0
                s = s-1
                while (s>0):
                    px = np.int32(s*math.cos(theta)+100)
                    py = np.int32(s*math.sin(theta)+100)
                    if (frame[px,py] == -1):
                        frame[px,py] = 1
                    s = s-1
                break
            s=s+1
            px = np.int32(s*math.cos(theta)+100)
            py = np.int32(s*math.sin(theta)+100)
                
                    
def Gauss_Seidel():
    grad = np.zeros(frame.shape)
    x = frame.shape[0]
    y = frame.shape[1]
    for i in range(1, x-1):
        for j in range(1, y-1):
            grad[i,j] = (-grad[i-1,j]+frame[i+1,j]-grad[i,j-1]+frame[i,j+1])/4.0
    grad = grad[1:x-1, 1:y-1]
    return grad

grad = Gauss_Seidel()
grad = grad - grad.min()
grad = grad/grad.max()
grad[grad>0.99] = 1

wh = np.where(grad == grad.max())
D = 0
X = 0
Y = 0
for i in range(0, wh[:][0].size):
    x = wh[0][i] - frame.shape[0]/2.0
    y = wh[1][i] - frame.shape[1]/2.0
    d = np.sqrt(x**2+y**2)
    if d>D:
        X = x
        Y = y
        D = d

theta = math.atan2(Y,X)

plt.imshow(grad, cmap='Greys')

img = np.uint8(255.0*(frame-frame.min())/(frame.max()-frame.min()))
img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

pt1 = (np.int32(np.mean(frame.shape[1]/2.0)), np.int32(np.mean(frame.shape[0]/2.0)))
pt2 = (np.int32(Y+pt1[0]),np.int32(X+pt1[1]))
img = cv2.line(img, pt1, pt2, (255,0,0),2)

plt.imshow(img)
