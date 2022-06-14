#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun  3 16:20:37 2022

@author: sanim
"""

import numpy as np

def eul2rotm(z, y, x):
    R = np.zeros((3,3))
    
    R[0,0] = np.cos(y)*np.cos(z)
    R[0,1] = np.sin(x)*np.sin(y)*np.cos(z) - np.cos(x)*np.sin(z)
    R[0,2] = np.cos(x)*np.sin(y)*np.cos(z) + np.sin(x)*np.sin(z)
    
    R[1,0] = np.cos(y)*np.sin(z)
    R[1,1] = np.sin(x)*np.sin(y)*np.sin(z) + np.cos(x)*np.cos(z)
    R[1,2] = np.cos(x)*np.sin(y)*np.sin(z) - np.sin(x)*np.cos(z)
    
    R[2,0] = -np.sin(y)
    R[2,1] = np.sin(x)*np.cos(y)
    R[2,2] = np.cos(x)*np.cos(y)
    
    return R

def quat2eul(w, x, y, z):
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)
    
    return X, Y, Z

def quat2rotm(w, x, y, z):
    angles = quat2eul(w, x, y, z)
    
    return eul2rotm(angles[2], angles[1], angles[0])
    
    
#rotation = eul2rotm(1.4915, -0.0166, -0.0055)
#quat = quat2rotm(-0.7346, -0.0036, 0.0079, -0.6785)
#print(rotation)