#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 16:02:41 2019

@author: kjchen
"""
import numpy as np
import math as m
import h5py


def load_data(extrinsics_fname, imperx_fname, ximea_fname):
    
    extrinsics, imperx, ximea = dict(), dict(), dict()

    # file paths
    extrinsics['fname'] = extrinsics_fname
    imperx['fname'] = imperx_fname
    ximea['fname'] = ximea_fname
    
    # load extrinsics data
    f = h5py.File(extrinsics['fname'], 'r')
    
    extrinsics['gt'] = np.zeros(shape=[4,4])
    extrinsics['gt'][0:3,0:3] = f['rvec'][()]
    extrinsics['gt'][0:3,3] = np.squeeze( f['tvec'][()] )
    extrinsics['gt'][3,:] = np.array([0, 0, 0, 1])

    # load imperx intrinsics data
    f = h5py.File(imperx['fname'], 'r')
    
    imperx['intrinsics'] = f['intrinsics'][()]
    imperx['fx'] = imperx['intrinsics'][0,0]; imperx['fy'] = imperx['intrinsics'][1,1];
    imperx['cx'] = imperx['intrinsics'][0,2]; imperx['cy'] = imperx['intrinsics'][1,2];
    
    imperx['d'] = f['dist'][()]
    
    # load ximea intrinsics data
    f = h5py.File(ximea['fname'], 'r')
    
    ximea['intrinsics'] = f['intrinsics'][()]
    ximea['fx'] = ximea['intrinsics'][0,0]; ximea['fy'] = ximea['intrinsics'][1,1];
    ximea['cx'] = ximea['intrinsics'][0,2]; ximea['cy'] = ximea['intrinsics'][1,2];
    
    ximea['d'] = f['dist'][()]
    
    return extrinsics, imperx, ximea
    

def solve_params(x, t, p, gt):
    
    # ximea frame to tilt frame
    t_T_x = np.array([[ 1,          0,          0,          x[0] ],
                      [ 0,          m.cos(t),  -m.sin(t),   x[1] ],
                      [ 0,          m.sin(t),   m.cos(t),   x[2] ],
                      [ 0,          0,          0,          1    ]], dtype='float64')
    
    # tilt frame to pan frame
    p_T_t = np.array([[ m.cos(p),   0,          m.sin(p),   x[3] ],
                      [ 0,          1,          0,          x[4] ],
                      [-m.sin(p),   0,          m.cos(p),   x[5] ],
                      [ 0,          0,          0,          1    ]], dtype='float64')
    
    # pan frame to imperx frame
    Rx = np.array([[ 1,             0,              0           ],
                   [ 0,             m.cos(x[9]),   -m.sin(x[9]) ],
                   [ 0,             m.sin(x[9]),    m.cos(x[9]) ]], dtype='float64')
    
    Ry = np.array([[ m.cos(x[10]),  0,              m.sin(x[10]) ],
                   [ 0,             1,              0            ],
                   [-m.sin(x[10]),  0,              m.cos(x[10]) ]], dtype='float64')
    
    Rz = np.array([[ m.cos(x[11]), -m.sin(x[11]),   0            ],
                   [ m.sin(x[11]),  m.cos(x[11]),   0            ],
                   [ 0,             0,              1            ]], dtype='float64')
    
    i_T_p = np.zeros(shape=[4,4], dtype='float64')
    i_T_p[0:3,0:3] = np.matmul( Rz, np.matmul( Ry, Rx ) )
    i_T_p[0:3,3] = np.array([ x[6], x[7], x[8] ])
    i_T_p[3,:] = np.array([0, 0, 0, 1])
    
    # ximea frame to imperx frame
    i_T_x = np.matmul( i_T_p, np.matmul( p_T_t, t_T_x ) )

    # fsolve return array
    F = np.zeros(12)
    F[0] = i_T_x[0,0] - gt[0,0]
    F[1] = i_T_x[0,1] - gt[0,1]
    F[2] = i_T_x[0,2] - gt[0,2]
    F[3] = i_T_x[0,3] - gt[0,3]
    F[4] = i_T_x[1,0] - gt[1,0]
    F[5] = i_T_x[1,1] - gt[1,1]
    F[6] = i_T_x[1,2] - gt[1,2]
    F[7] = i_T_x[1,3] - gt[1,3]
    F[8] = i_T_x[2,0] - gt[2,0]
    F[9] = i_T_x[2,1] - gt[2,1]
    F[10] = i_T_x[2,2] - gt[2,2]
    F[11] = i_T_x[2,3] - gt[2,3]

    return F


def dynamic_extrinsics(x, t, p):
    
    tf = dict()
    
    # ximea frame to tilt frame
    tf['t_T_x'] = np.array([[ 1,        0,          0,          x[0] ],
                            [ 0,        m.cos(t),  -m.sin(t),   x[1] ],
                            [ 0,        m.sin(t),   m.cos(t),   x[2] ],
                            [ 0,        0,          0,          1    ]], dtype='float64')
    
    # tilt frame to pan frame
    tf['p_T_t'] = np.array([[ m.cos(p),  0,          m.sin(p),   x[3] ],
                            [ 0,         1,          0,          x[4] ],
                            [-m.sin(p),  0,          m.cos(p),   x[5] ],
                            [ 0,         0,          0,          1    ]], dtype='float64')
    
    # pan frame to imperx frame
    Rx = np.array([[ 1,                 0,                  0               ],
                   [ 0,                 m.cos(x[9]),       -m.sin(x[9])     ],
                   [ 0,                 m.sin(x[9]),        m.cos(x[9])     ]], dtype='float64')
    
    Ry = np.array([[ m.cos(x[10]),      0,                  m.sin(x[10])    ],
                   [ 0,                 1,                  0               ],
                   [-m.sin(x[10]),      0,                  m.cos(x[10])    ]], dtype='float64')
    
    Rz = np.array([[ m.cos(x[11]),     -m.sin(x[11]),       0               ],
                   [ m.sin(x[11]),      m.cos(x[11]),       0               ],
                   [ 0,                 0,                  1               ]], dtype='float64')
    
    tf['i_T_p'] = np.zeros(shape=[4,4], dtype='float64')
    tf['i_T_p'][0:3,0:3] = np.matmul( Rz, np.matmul( Ry, Rx ) )

    tf['i_T_p'][0:3,3] = np.array([ x[6], x[7], x[8] ])
    tf['i_T_p'][3,:] = np.array([0, 0, 0, 1])
    
    # ximea frame to imperx frame
    tf['i_T_x'] = np.matmul( tf['i_T_p'], np.matmul( tf['p_T_t'], tf['t_T_x'] ) )
    
    return tf


def compute_world(tf, ximea, imperx, uc, vc, zc):
    
    # transform from ximea image frame to ximea camera frame
    xc = ( (uc - ximea['cx']) * zc ) / ximea['fx']
    yc = ( (vc - ximea['cy']) * zc ) / ximea['fy']
    
    # transform from ximea camera frame to imperx camera frame
    xyz_c = np.array([xc, yc, zc, 1])
    xyz_w = np.matmul( tf['i_T_x'], xyz_c )
    
    # transform from imperx camera frame to imperx image frame
    uw = ( (xyz_w[0] * imperx['fx']) / xyz_w[2] ) + imperx['cx']
    vw = ( (xyz_w[1] * imperx['fy']) / xyz_w[2] ) + imperx['cy']
    
    return xyz_w[0:3], np.array([uw, vw])


