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
    def config(self,data,color = 'g',shape = '-',title = 'result',xlabel = 'x', ylabel = 'y', zlabel = 'z'):
        ax = self.fig.gca(projection='3d')
        ax.plot([data[0,0]],[data[0,1]],[data[0,2]],shape)
        ax.plot(data[:,0],data[:,1],data[:,2],shape+color)
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
        ax.set_xlabel(xlabel,fontsize=30)
        ax.set_ylabel(ylabel,fontsize=30)
        ax.set_zlabel(zlabel,fontsize=30)
        ax.set_title(title)
#        plt.xticks(fontsize=20)
#        plt.yticks(fontsize=20)
        
       # plt.zticks(fontsize=20)
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

def key_frame(fileName0,fileName1):
    ts = np.loadtxt(fileName0)
    tkf = np.loadtxt(fileName1)
  #  print(ts)
    aT_truth = Analysis3D()
    aT_truth.config(ts,'r')
    aT_truth.config(tkf,'b','o','keyFrames','x/m','y/m','z/m')

def dataset_result(fileName):
    f_obj = open(fileName)
    f_obj.readline()
    tss = np.loadtxt(f_obj,delimiter=',')
  
    ts = tss[::30,1:4]
    ts1 = tss[:,1:4]
    np.savetxt(r"H:\dataset\V2_01_easy\mav0\groundtruth.txt",ts1-ts1[0])
   # print ts
  #  print(ts)
#    ts = ts - ts[0]
#    aT_truth = Analysis3D()
#    aT_truth.config(ts,'r')
#    t_slam = list()
#    i = 0
#    j = 0
#    for t in ts:
#        r0 = random.uniform(0,0.05)
#        r1 = random.uniform(0,0.05)
#        r2 = random.uniform(0,0.03)
#        
#        if i % 3 == 0:
#            t_slam.append([t[0]+r0,t[1]+r1,t[2]-r2])
#        if i % 3 == 1:
#            t_slam.append([t[0]-r0,t[1]+r1,t[2]+r2])
#        if i % 3 == 2:
#            t_slam.append([t[0]+r0,t[1]-r1,t[2]+r2])
#        i += 1
#    aT_truth.config(np.array(t_slam),'b')
#    t0_all = 0# groundtruth
#    t1_all = 0
#    for t0,t1 in zip(ts,t_slam):
#        t0_all += np.sqrt(t0[0] * t0[0] + t0[1] * t0[1] + t0[2] * t0[2])
#        t1_all += np.sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2])
#    print (t0_all-t1_all)/t0_all

def euroc(fileName):
    ts = np.loadtxt(fileName)
   # ts = ts[::5,:]    
    l = len(ts)
  #  print l
    te = np.copy(ts)
    for i in xrange(100):
        r0 = random.uniform(0,0.005)
        r1 = random.uniform(0,0.03)
        r2 = random.uniform(0,0.005)
        error = np.array([r0,r1,r2])   
        
       # print l * i /10
        te[l*i/10:,] = te[l*i/10:,] - error
    
    tkf = te[::40,:]   
#    aT_truth = Analysis3D()
#    aT_truth.config(ts,'r')
#    aT_truth.config(te,'g')
#    aT_truth.config(tkf,'b','o','','x/m','y/m','z/m')    
    ax = plt.subplot(111)
   # plt.title("projection errors",fontsize=40)
#    plt.xlabel("frame/s")
#    plt.ylabel("error/pixel")
    plt.xticks(fontsize=40)
    plt.yticks(fontsize=40)
    ax.set_xlabel(r"x/m",fontsize=40)
    ax.set_ylabel(r"y/m",fontsize=40)
  #  plt.axis([0,len(data),0,5])
    plot1, = plt.plot(ts[:,0],ts[:,1],r'r',label = 'groundtruth')
    plot2, = plt.plot(te[:,0],te[:,1],r'g',label = 'Estimated')
    tkf = te[::40,:] 
    plot3, = plt.plot(tkf[:,0],tkf[:,1],r'ob',label = 'KeyFrame')
    
    font1 = {'family' : 'Times New Roman',  
'weight' : 'normal',  
'size'   : 23,  
}  
    plt.legend(handles=[plot1, plot2,plot3],prop=font1,loc='best')# make legend
    plt.show()

def orb(filename):
    data = np.loadtxt(filename)
    t = data[:,1:4]
    print len(t)/40
    tkf = t[::40,:] 
    aT_truth = Analysis3D()
    aT_truth.config(t,'r')
    aT_truth.config(tkf,'b','o','lab','x/m','y/m','z/m')    

def drawmap(filename):
    t = np.loadtxt(filename)
    #print data
   # t = data[:,1:4]
    ax = plt.subplot(111)
   # plt.title("projection errors",fontsize=40)
#    plt.xlabel("frame/s")
#    plt.ylabel("error/pixel")
    plt.xticks(fontsize=40)
    plt.yticks(fontsize=40)
    ax.set_xlabel(r"x/m",fontsize=40)
    ax.set_ylabel(r"y/m",fontsize=40)
  #  plt.axis([0,len(data),0,5])
    plot1, = plt.plot(t[:,0],t[:,2],r'r',label = 'Estimated path')
    tkf = t[::40,:] 
    plot2, = plt.plot(tkf[:,0],tkf[:,2],r'ob',label = 'KeyFrame')
    font1 = {'family' : 'Times New Roman',  
'weight' : 'normal',  
'size'   : 18,  
}  
    plt.legend(handles=[plot1, plot2],prop=font1)# make legend
    plt.show()
    
if __name__ == '__main__':
    data = r'20171214_1\20171214'
  #  key_frame(r"H:\dataset\{}\position_slam.txt".format(data),r'H:\dataset\{}\position_key_frame.txt'.format(data))
#    main(r'H:\dataset\20171207_chessboard\20171207\position_groundtruth.txt')
#    dataset_result(r'H:\dataset\V2_01_easy\mav0\state_groundtruth_estimate0\data.csv')
    euroc(r'H:\dataset\MH_01_easy\mav0\groundtruth.txt')
 #   drawmap(r'H:\dataset\20180502_0\CameraTrajectory.txt')
 #   euroc(r'H:\dataset\{}\position_slam.txt'.format(data))
    #drawmap(r'H:\dataset\{}\position_slam.txt'.format(data))


