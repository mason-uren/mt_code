#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File:  ccm.py  -- for "Compute Cad Model"  CAD Model is a misnomer, inherited from DK

Code translated from DK's MATLAB code: compute_T_matrix.m and compute_extrinsics.m

Created on Thu Jun 13 18:33:10 PDT 2019

@author: ychen
"""
import numpy as np
import math as m
from Dynamic_Extrinsics.Main.Deepaks_Model import matrices
import json


# TODO: put parameters in a namedtuple

# Translated from Deepak's code
# yaw/pitch/roll all in radians
def compute_T_matrix(yaw, pitch, roll, tx, ty, tz):
    Rx = np.array([[1, 0, 0],
                   [0, m.cos(pitch), -m.sin(pitch)],
                   [0, m.sin(pitch), m.cos(pitch)]], dtype='float64')
    Ry = np.array([[m.cos(yaw), 0, m.sin(yaw)],
                   [0, 1, 0],
                   [-m.sin(yaw), 0, m.cos(yaw)]], dtype='float64')
    Rz = np.array([[m.cos(roll), -m.sin(roll), 0],
                   [m.sin(roll), m.cos(roll), 0],
                   [0, 0, 1]], dtype='float64')

    Rot_matrix = np.matmul(Rz, np.matmul(Ry, Rx))
    T_matrix_3x4 = np.hstack((Rot_matrix, np.array([[tx], [ty], [tz]], dtype='float64')))
    T_matrix_4x4 = np.vstack((T_matrix_3x4, np.array([0, 0, 0, 1], dtype='float64')))

    # print('yaw, pitch,roll, tx, ty, tz =\n{}'.format((yaw, pitch,roll, tx, ty, tz)))
    # print('Rx=\n{}'.format(Rx))
    # print('Ry=\n{}'.format(Ry))
    # print('Ry*Rx=\n{}'.format(np.matmul(Ry,Rx)))
    # print('Rz=\n{}'.format(Rz))
    # print('Rot_matrix=\n{}'.format(Rot_matrix))
    # print('Trans_matrix=\n{}'.format(T_matrix_4x4))

    return T_matrix_4x4


# Compute the forward extrinsics transformation matrix Ximea-->ImperX using
# Deepak's 15-parameter model stored in 'cad_model'.
# tilt & pan: tilt and pan angles of the P/T unit in radian & corrected for 90-deg in
# right-hand-rule (RHL) coordinate system; i.e., tilt up from level is positive,
# down is negative; pan to the right is positive, to the left is negative.
# This is probably different than clm.dynamic_extrinsics() from Kenny Chen

def dynamic_extrinsics_dk15params(cad_model, tilt, pan):
    # Extract the values from cad_model:
    # Numerical Extrinsics
    # Ximea to Tilt
    yaw_ximea_tilt = cad_model[0]
    pitch_ximea_tilt = cad_model[1]
    roll_ximea_tilt = cad_model[2]
    x_ximea_tilt = cad_model[3]
    y_ximea_tilt = cad_model[4]
    z_ximea_tilt = cad_model[5]
    # Tilt to Pan
    pitch_tilt_pan = tilt
    x_tilt_pan = cad_model[6]
    y_tilt_pan = cad_model[7]
    z_tilt_pan = cad_model[8]
    # Pan to Base
    yaw_pan_base = pan;
    # Base to Imperx
    yaw_base_imperx = cad_model[9]
    pitch_base_imperx = cad_model[10]
    roll_base_imperx = cad_model[11]
    x_base_imperx = cad_model[12]
    y_base_imperx = cad_model[13]
    z_base_imperx = cad_model[14]
    # Ximea to Tilt (ximea_tilt)
    T_ximea_tilt = compute_T_matrix(yaw_ximea_tilt, pitch_ximea_tilt, roll_ximea_tilt,
                                    x_ximea_tilt, y_ximea_tilt, z_ximea_tilt)
    #   print('T_ximea_tilt=\n{}'.format(T_ximea_tilt))
    # Tilt to Pan (tilt_pan)
    T_tilt_pan = compute_T_matrix(0, pitch_tilt_pan, 0, x_tilt_pan, y_tilt_pan, z_tilt_pan)
    # Pan to Base (pan_base)
    T_pan_base = compute_T_matrix(yaw_pan_base, 0, 0, 0, 0, 0);
    # Base to Imperx (base_imperx)
    T_base_imperx = compute_T_matrix(yaw_base_imperx, pitch_base_imperx, roll_base_imperx,
                                     x_base_imperx, y_base_imperx, z_base_imperx)
    # Full transformation Ximea to Imperx (ximea_imperx)
    #  ???  T_ximea_imperx = T_ximea_tilt*T_tilt_pan*T_pan_base*T_base_imperx
    T_ximea_imperx = np.matmul(T_ximea_tilt, np.matmul(T_tilt_pan, np.matmul(T_pan_base, T_base_imperx)))
    # T_ximea_imperx = np.matmul(T_base_imperx,np.matmul(T_pan_base,np.matmul(T_tilt_pan,T_ximea_tilt)))

    return T_ximea_imperx


def dynamic_extrinsics_correct_order(cad_model, tilt, pan):
    # Extract the values from cad_model:
    # Numerical Extrinsics
    # Ximea to Tilt consists of an arbitrary translation and rotation
    yaw_ximea_tilt = cad_model[0]
    pitch_ximea_tilt = cad_model[1]
    roll_ximea_tilt = cad_model[2]
    x_ximea_tilt = cad_model[3]
    y_ximea_tilt = cad_model[4]
    z_ximea_tilt = cad_model[5]

    r_ximea_tilt = matrices.rotation(pitch_ximea_tilt, yaw_ximea_tilt, roll_ximea_tilt)
    t_ximea_tilt = matrices.translation(x_ximea_tilt, y_ximea_tilt, z_ximea_tilt)

    # Tilt to Pan is a pitch followed by a translation
    pitch_tilt_pan = tilt
    x_tilt_pan = cad_model[6]
    y_tilt_pan = cad_model[7]
    z_tilt_pan = cad_model[8]

    r_tilt_pan = matrices.rotation(pitch_tilt_pan, 0, 0)
    t_tilt_pan = matrices.translation(x_tilt_pan, y_tilt_pan, z_tilt_pan)

    # Pan to Base is simply a pan (yaw) rotation
    yaw_pan_base = pan;

    r_pan_base = matrices.rotation(0, yaw_pan_base, 0)

    # Base to Imperx is another arbitrary translation and rotation
    yaw_base_imperx = cad_model[9]
    pitch_base_imperx = cad_model[10]
    roll_base_imperx = cad_model[11]
    x_base_imperx = cad_model[12]
    y_base_imperx = cad_model[13]
    z_base_imperx = cad_model[14]

    r_base_imperx = matrices.rotation(pitch_base_imperx, yaw_base_imperx, roll_base_imperx)
    t_base_imperx = matrices.translation(x_base_imperx, y_base_imperx, z_base_imperx)

    return r_base_imperx @ t_base_imperx @ r_pan_base @ t_tilt_pan @ r_tilt_pan @ t_ximea_tilt @ r_ximea_tilt


# Tester
if __name__ == '__main__':
    cad_model = []
    # with open('cad_models/cad_model.json', 'r') as f:
    #     cad_model = json.load(f)
    with open('cad_models/cad_model2.json', 'r') as f:
        cad_model = json.load(f)
    cadmdl = cad_model['cad_model']
    # print('cad_model= {}'.format(cadmdl))
    x = [cadmdl['yaw_ximea_tilt'],
         cadmdl['pitch_ximea_tilt'],
         cadmdl['roll_ximea_tilt'],
         cadmdl['x_ximea_tilt'],
         cadmdl['y_ximea_tilt'],
         cadmdl['z_ximea_tilt'],
         cadmdl['x_tilt_pan'],
         cadmdl['y_tilt_pan'],
         cadmdl['z_tilt_pan'],
         cadmdl['yaw_base_imperx'],
         cadmdl['pitch_base_imperx'],
         cadmdl['roll_base_imperx'],
         cadmdl['x_base_imperx'],
         cadmdl['y_base_imperx'],
         cadmdl['z_base_imperx'],
         ]
    # these 2 sets of pan/tilt are used for computing "Extrinsics_TV_High_Quality_Checked_by_Yang.h5"
    # from which the parameters in cad_model.json are derived from (DK's mainlime.m code)
    # Raw pan/tilt #1
    pan = 84.7678  # deg
    tilt = 6.0777  # deg
    print('dynamic extrinsics for (pan, tilt)={} (deg, raw)'.format((pan, tilt)))
    # Convert to RHL, radians
    pan = m.radians(90.0 - pan)
    tilt = m.radians(-tilt)

    # print(dynamic_extrinsics_dk15params(x, tilt, pan))
    print(dynamic_extrinsics_correct_order(x, tilt, pan))

    # Raw pan/tilt #2
    pan = 94.3614  # deg
    tilt = 6.1870  # deg
    print('dynamic extrinsics for (pan, tilt)={} (deg, raw)'.format((pan, tilt)))
    # Convert to RHL, radians
    pan = m.radians(90.0 - pan)
    tilt = m.radians(-tilt)

    # print(dynamic_extrinsics_dk15params(x, tilt, pan))
    print(dynamic_extrinsics_correct_order(x, tilt, pan))
