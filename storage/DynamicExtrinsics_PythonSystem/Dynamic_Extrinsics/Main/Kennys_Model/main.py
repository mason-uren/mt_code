#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 16:02:34 2019

@author: kjchen
"""

import numpy as np
import math as m
from scipy.optimize import least_squares
import h5py

#%% Train Parameters

TRAIN_TILT_DEG = np.array([8.107199668884277,
                           8.49959945678711,
                           8.699399948120117,
                           9.200699806213379,
                           9.999449729919434])

TRAIN_PAN_DEG = np.array([76.89644622802734,
                          77.99939727783203,
                          78.99974822998047,
                          80.00054931640625,
                          80.99954986572266])
    
TRAIN_PAN_DEG = -(90 - TRAIN_PAN_DEG)

xn = np.zeros(shape=(12,5))

for n in range(0,5):

    # load data
    extrinsics_fname = './train/extrinsics_' + str(n+1) + '.h5'
    imperx_fname = './train/imperx_intrinsics.h5'
    ximea_fname = './train/ximea_intrinsics.h5'
    
    extrinsics, imperx, ximea = clm.load_data(extrinsics_fname, imperx_fname, ximea_fname)

    # initial values
    x0 = np.array([0.5, 0.0, -0.1, -0.2, 0.4, 0.0, 0.75, -0.2, 0.0, 0.0, 0.0, 0.0])
    
    # tilt and pan angles
    t = m.radians(TRAIN_TILT_DEG[n])
    p = m.radians(TRAIN_PAN_DEG[n])
    
    # solve system of nonlinear equations
    params = least_squares(clm.solve_params, x0, args=(t, p, extrinsics['gt']), method='dogbox', tr_solver='exact')
    
    xn[:,n] = params['x']
    
# average parameters
x = xn.mean(1)
np.save("x.npy",x)

#%% Dynamic Extrinsicsround1

# tilt and pan angles
CURR_TILT_DEG = 8.271900177001953
CURR_PAN_DEG = -(90 - 81.18585205078125)

# compute the extrinsics matrix for the specified tilt and pan angles
tf = clm.dynamic_extrinsics(x, t=m.radians(CURR_TILT_DEG), p=m.radians(CURR_PAN_DEG))

# read test data
f = h5py.File('./train/test_ximea_left.h5', 'r')
uv1 = f['point_0'][()]
tvec1 = f['tvec'][()]

f = h5py.File('./train/test_ximea_right.h5', 'r')
uv2 = f['point_0'][()]
tvec2 = f['tvec'][()]

# compute world coordinates
xyz_w1, uv_w1 = clm.compute_world(tf, ximea, imperx, uc=uv1[0], vc=uv1[1], zc=tvec1[2,0])