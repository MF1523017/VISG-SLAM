#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 13:44:18 2017

@author: LiPei
"""


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import sys
import random

class Analysis3D(object):
    def __init__(self):
        self.fig=plt.figure()
    def config(self,data,color = 'g'):
        ax = self.fig.gca(projection='3d')
        ax.plot([data[0,0]],[data[0,1]],[data[0,2]],'-')
        ax.plot(data[:,0],data[:,1],data[:,2],'-'+color)
        ax.legend()
#        xMin = min(data[:,0])
#        xMax = max(data[:,0])
#        yMin = min(data[:,1])
#        yMax = max(data[:,1])
#        zMin = min(data[:,2])
#        zMax = max(data[:,2])
#        setMinV = min(xMin,yMin,zMin)
#        setMaxV = max(xMax,yMax,zMax)
#        scale = len(data)
#        ax.set_xlim(setMinV-(xMax-xMin)/scale,setMaxV+(xMax-xMin)/scale)
#        ax.set_ylim(setMinV-(yMax-yMin)/scale,setMaxV+(yMax-yMin)/scale)
#        ax.set_zlim(setMinV-(zMax-zMin)/scale,setMaxV+(zMax-zMin)/scale)
#        ax.set_xlim(xMin-(xMax-xMin)/scale,xMax+(xMax-xMin)/scale)
#        ax.set_ylim(yMin-(yMax-yMin)/scale,yMax+(yMax-yMin)/scale)
#        ax.set_zlim(zMin-(zMax-zMin)/scale,zMax+(zMax-zMin)/scale)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.set_title('result')
        plt.show()


def main(fileName):
    ts = np.loadtxt(fileName)
  #  print(ts)
    aT_truth = Analysis3D()
    aT_truth.config(ts,'r')
#    t_slam = list()
#    i = 0
#    for t in ts:
#        r0 = random.uniform(0,0.003)
#        r1 = random.uniform(0,0.002)
#        r2 = random.uniform(0,0.003)
#        if i % 3 == 0:
#            t_slam.append([t[0]+r0,t[1]+r1,t[2]-r2])
#        if i % 3 == 1:
#            t_slam.append([t[0]-r0,t[1]+r1,t[2]+r2])
#        if i % 3 == 2:
#            t_slam.append([t[0]+r0,t[1]-r1,t[2]+r2])
#    
#    aT_truth.config(np.array(t_slam),'b')
#    t0_all = 0# groundtruth
#    t1_all = 0
#    for t0,t1 in zip(ts,t_slam):
#        t0_all += np.sqrt(t0[0] * t0[0] + t0[1] * t0[1] + t0[2] * t0[2])
#        t1_all += np.sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2])
#    print (t0_all-t1_all)/t0_all
if __name__ == '__main__':
    main(r"H:\dataset\20180424_0\position_slam.txt")
#    main(r'cur_origin_new.txt')



