'''
Rename file to `config` or something
'''

import os
import numpy as np

# Paths to intrinsics and distortion files
rootdir = '../../../src/Metrology2020_VS/Metrology2020/Config/CameraIntrinsics'
imperxdir = 'Imperx_id2/2020-06-26'
ximeadir = 'Ximea_id0/2020-06-19'

# Each of the matrices here are [3, 3] representing the intrinsics projections.
intrinsics = dict(
    ximea=np.load(os.path.join(rootdir, ximeadir, 'ximea_intrinsics.npy')),
    imperx=np.load(os.path.join(rootdir, imperxdir, 'imperx_intrinsics.npy'))
)

# Each array here should be [5] distortion coefficients unless we upgrade
# to a more complicated distortion model.
distortion = dict(
    ximea=np.load(os.path.join(rootdir, ximeadir, 'ximea_distCoeffs.npy')),
    imperx=np.load(os.path.join(rootdir, imperxdir, 'imperx_distCoeffs.npy'))
)
