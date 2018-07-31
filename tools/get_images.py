# -*- coding: utf-8 -*-
"""
Created on Tue Sep 26 10:56:38 2017

@author: lipei
"""

import os


def nums(f):
    name,_ = f.split('.')
    return int(name)

def get_images(file_dir):
    files = os.listdir(os.path.join(file_dir,'data'))
 #   print (files)
    filess = sorted(files,key = nums)
 #   print filess
    with open(os.path.join(file_dir,'data.csv'),'w') as data:
        for f in filess:    
            name,_ = f.split('.')
            data.write(name)
            data.write(',')
            data.write(f)
            data.write(" \n")

if __name__ == '__main__':
    file_dir = r'H:\dataset\MH_01_easy\mav0\cam0'
    get_images(file_dir)