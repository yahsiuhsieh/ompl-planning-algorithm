# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 01:02:46 2020

@author: coldhenry
"""
import numpy as np

def read2path(file):
    path = []
    f = open(file,'r')
    line = f.readline()
    
    while line:
        
        line = line.strip('[')
        line = line[:-2]
        info = line.split(' ')
        coor = []
        for i in info:
            coor.append(i)
        path.append(coor)
        line = f.readline()
        
    return np.array(path)


if __name__ == '__main__':
    
    filename = './data/rrt_flappy.txt'
    path = read2path(filename)