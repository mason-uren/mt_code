'''
TODO: deprecate. Replaced by `extrinsics.py`

Given a model for relative extrinsics and a dataset of measurements from both
Ximea and Imperx, the functions provided in this file compute various error
metrics on the dataset.
'''

import numpy as np
import pandas as pd
from scipy.stats import circstd
import matrices
from transforms import apply_rigid
from board import board

# TODO: move these functions
# TODO: does opencv provide a function that does this?
def get_chessboard_corners(poses):
    '''
    Take a board's pose as a [B, 4, 4] ndarray, return the 3d positions of each of
    its chessboard corners as an [B, N, 3] ndarray.
    '''
    rot_mat = poses[:, 0:3, 0:3]
    tln_vec = poses[:, 0:3, 3]
    
    # TODO: This is basically a reimplementation of apply_rigid
    #corners = (rot_mat @ board.chessboardCorners.T).swapaxes(1, 2)
    #corners += tln_vec[:, np.newaxis, :]
    corners = board.chessboardCorners @ rot_mat.swapaxes(1, 2) + tln_vec[:, np.newaxis, :]
   
    return corners
    

def get_imgpoints(corners):
    return corners[..., 0:2] / corners[..., [2]]
    
def get_ximea_corners(model, rvec, tvec, dataset):
    '''
    From a fiducial pose in world space represented by `rvec` and `tvec`,
    get its corners in 3d coordinates in ximea space.
    '''
    
    # Get extrinsics matrix represented by rvec and tvec
    world_exts = apply_rigid(rvec, tvec, np.eye(4)[np.newaxis, :, :])
    world_exts = np.repeat(world_exts, dataset['n_records'], axis=0)
    
    # Transform into ximea space
    ximea_exts = model.apply(pans=dataset['pan'],
                             tilts=dataset['tilt'],
                             world_exts=world_exts)

    ximea_corners = get_chessboard_corners(ximea_exts)
    
    return ximea_corners
    
def dynamic_exploded_loss(model, rvec, tvec, dataset):
    '''
    Computes the average error between the corners in 3d measured by ximea and
    those represented by rvec and tvec.
    '''
    true_ximea_corners = get_ximea_corners(model, rvec, tvec, dataset)
    est_ximea_corners = get_chessboard_corners(dataset['ximea_ext'])
    
    corner_errs = np.linalg.norm(est_ximea_corners - true_ximea_corners, axis=2)
    return np.mean(corner_errs)
    
def static_exploded_norms(model, dataset):
    '''
    Computes the average error between the corners in 3d measured by ximea and
    those measured by imperx.
    '''
    ximea_est_exts = model.apply_inverse(pans=dataset['pan'],
                                         tilts=dataset['tilt'],
                                         local_exts=dataset['ximea_ext'])
    imperx_est_exts = dataset['impx_ext']

    ximea_est_corners = get_chessboard_corners(ximea_est_exts)
    imperx_est_corners = get_chessboard_corners(imperx_est_exts)
    
    corner_errs = np.linalg.norm(ximea_est_corners - imperx_est_corners, axis=2)
    
    return np.mean(corner_errs, axis=1)
    
def static_exploded_loss(model, dataset):
    return np.mean(static_exploded_norms(model, dataset))

#--------------Added by Yang Chen------------------------------
def static_robot_pt_norms(model, dataset):
    '''
    Computes the average error between the robot manipulator (tool/print-head) in 3d measured 
    by ximea and those measured by imperx, when the relative pose of robot manipulator
    to the fiducial is represented by an extra 4x4 matrix, robot_ext.
    '''
    from robot_manipulator import robot_pt_ext

    # Robot point manipulator in Ximea camera frames:
    ximea_robot_pt_exts = [ ext @ robot_pt_ext for ext in dataset['ximea_ext']]

    # transformed to Imperx (World) frame:
    ximea_est_exts = model.apply_inverse(pans=dataset['pan'],
                                         tilts=dataset['tilt'],
                                         local_exts=ximea_robot_pt_exts)

    # Robot point manipulator in Imperx (world)  frames:
    imperx_est_exts = [ ext @ robot_pt_ext for ext in dataset['impx_ext']]

    # Extract the translation component of the 4x4, and calc the err:
    pt_errs = []
    for ximea_ext, imperx_ext in zip(ximea_est_exts,imperx_est_exts):
        pt_errs.append(ximea_ext[0:3, 3] - imperx_ext[0:3, 3])

    #print(pt_errs)
    errs = np.linalg.norm(pt_errs, axis=1)
    
    return errs

def static_robot_pt_loss(model, dataset):
    return np.mean(static_robot_pt_norms(model, dataset))
    
#-------End of -- Added by Yang Chen------------------------------

def dynamic_reprojection_loss(model, rvec, tvec, dataset):
    '''
    Average error between the corners in 2d pixel space measured by ximea and
    those represented by rvec and tvec.
    '''
    
    # Second argument is the corners that come from solvepnp approach
    true_ximea_corners = get_ximea_corners(model, rvec, tvec, dataset)
    true_ximea_imgpoints = get_imgpoints(true_ximea_corners)
    
    est_ximea_imgpoints = dataset['UV_corners_ximea']

    imgpoint_errs = np.linalg.norm(est_ximea_imgpoints - true_ximea_imgpoints, axis=2)

    # nanmean here because some of the corners may not have been detected
    return np.mean(np.nanmean(imgpoint_errs, axis=1))

def static_reprojection_loss(model, dataset):
    '''
    Average error between the corners in 2d pixel space measured by imperx and
    those predicted by the pose measured by ximea.
    '''
    predicted_exts = model.apply_inverse(pans=dataset['pan'],
                                      tilts=dataset['tilt'],
                                      local_exts=dataset['ximea_ext'])
    predicted_imgpoints = get_imgpoints(get_chessboard_corners(predicted_exts))
    measured_imgpoints = dataset['UV_corners_imperx']

    imgpoint_errs = np.linalg.norm(predicted_imgpoints - measured_imgpoints, axis=2)
    
    return np.mean(np.nanmean(imgpoint_errs, axis=1))
