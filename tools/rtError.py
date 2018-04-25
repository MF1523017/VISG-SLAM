#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 15:45:40 2017

@author: pp
"""


import numpy as np
import matplotlib.pyplot as plt
import sys
import os
plt.figure(figsize=(12,8),dpi=80)
ax1 = plt.subplot(411) 
ax2 = plt.subplot(412)
ax3 = plt.subplot(413)
ax4 = plt.subplot(414)

def getData(fileName):
    f = open(fileName, "r")
    f.readline()
    value = f.readlines()
    return np.loadtxt(value)

def config(error,color = 'g',method = ''):
    x = range(len(error))
    angleRMSE = error[:,1]
#    pdb.set_trace()
    relativeAngleRMSE = error[:,2]
    distanceRMSE = error[:,3]
    relativeDistanceRMSE = error[:,4]
    ax1.set_title("RMSE") 
    ax1.plot(x,angleRMSE,"-"+color,label=method+" : angle RMSE")
    ax1.legend()
    ax2.plot(x,relativeAngleRMSE,"-"+color,label=method+" : relative Angle RMSE")
    ax2.axis([0,len(x),0,5])
    ax2.legend()
    ax3.plot(x,distanceRMSE,"-"+color,label=method+" : distance RMSE")
    ax3.legend()
    ax4.plot(x,relativeDistanceRMSE,"-"+color,label=method+" : relative Distance RMSE")
    ax4.axis([0,len(x),0,5])
    ax4.legend()
    
def draw():
    colors = ['r','g','b','c','m','y','k','w']
    for i in xrange(1,len(sys.argv)):
        fileName = r'{}_result.txt'.format(sys.argv[i])
        error = getData(fileName)
        p,f = os.path.split(sys.argv[i])
        method = '_'.join(f.split('_')[:2])
        config(error,colors[(i-1)%8],method)
    plt.show()
    
def DrawProjectError(file_name):
    errors = np.loadtxt(file_name)
    plt.figure(2)
    plt.plot(errors)
    plt.axis([0,len(errors),0,5])
    plt.title("projection errors")
    plt.xlabel("frame")
    plt.ylabel("error")
    plt.show()

if __name__ == '__main__':
#    if(len(sys.argv) == 1):
#        print("usage:python {} prefix ".format(sys.argv[0]))
#    draw()
    file_name = r'H:\dataset\20180424_0\errors.txt'
    DrawProjectError(file_name)