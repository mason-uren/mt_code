"""
Provides functions for getting common 4x4 transformation matrices from their
parameters, and performing common operations on 4x4 extrinsics matrices.
"""

import numpy as np
import numba as nb
from scipy.spatial.transform import Rotation
from transforms3d.euler import euler2mat, mat2euler

ROT_IDXS = np.ix_([0,1,2],[0,1,2])
TLN_IDXS = ([0,1,2], 3)
ROT_ORDER = 'xyz'

@nb.njit()
def apply_tilts(tilts, exts):
    res_exts = exts.copy()
    for i in nb.prange(exts.shape[0]):
        tilt = tilts[i]
        ext = exts[i]
        c = np.cos(tilt)
        s = np.sin(tilt)
        row1 = c * ext[1] - s * ext[2]
        row2 = s * ext[1] + c * ext[2]
        res_exts[i][1] = row1
        res_exts[i][2] = row2
        
    return res_exts

@nb.njit()
def apply_pans(pans, exts):
    res_exts = exts.copy()
    for i in nb.prange(exts.shape[0]):
        pan = pans[i]
        ext = exts[i]
        c = np.cos(pan)
        s = np.sin(pan)
        row0 =  c * ext[0] + s * ext[2]
        row2 = -s * ext[0] + c * ext[2]
        res_exts[i][0] = row0
        res_exts[i][2] = row2
        
    return res_exts
    
def rvec_from_ext_mat(ext_mat):
    """
    Returns a vector of the rotation component of a 4x4 transformation
    expressed in Euler angles in the order (pitch, yaw, roll).
    """
    return mat2euler(ext_mat[ROT_IDXS])
    
def tvec_from_ext_mat(ext_mat):
    """
    Returns a vector of the translation component of a 4x4 transformation
    """
    return ext_mat[TLN_IDXS]
    
def vecs_from_ext_mat(ext_mat, degrees=True, mm=True):
    """
    Returns the rotation and translation vectors which give rise to ext_mat
    stacked into a 3x2 array (rotation, then translation).
    """
    rvec = rvec_from_ext_mat(ext_mat)
    if degrees: rvec = np.degrees(rvec)
        
    tvec = tvec_from_ext_mat(ext_mat)
    if mm: tvec = 1000 * tvec
        
    return np.array([rvec, tvec]).T

def vecs_from_ext_mats(ext_mats):
    """
    TODO: docstring
    """
    n_exts = ext_mats.shape[0]
    
    vecs = np.zeros((n_exts, 3, 2))
    for i in range(n_exts):
        vecs[i] = vecs_from_ext_mat(ext_mats[i])
        
    return vecs

def rotation(*, roll=0, pitch=0, yaw=0):
    """
    Returns a 4x4 matrix representing a translation in 3d. rx, ry, and rz
    correspond to pitch, yaw, and roll respectively.
    """
    rmat = np.eye(4)
    rmat[ROT_IDXS] = euler2mat(pitch, yaw, roll)
        
    return rmat

def translation(*, x=0, y=0, z=0):
    """
    Returns a 4x4 matrix representing a translation in 3d.
    """
    tmat = np.eye(4)
    tmat[TLN_IDXS] = [x, y, z]

    return tmat

# Tester
if __name__ == '__main__':
    print(translation(x=2,y=3,z=4))
    print(rotation(pitch=1,yaw=2,roll=3))
