'''
Rename file to `config` or something
'''

import os
import numpy as np

# Paths to intrinsics and distortion files
rootdir = '../../../src/Config/CameraIntrinsics'
ximeadir = 'Ximea_id0/2020-07-15'
ximea_cameraMatrix=os.path.join(rootdir, ximeadir, 'ximea_cameraMatrix.npy')
ximea_distCoeffs=os.path.join(rootdir, ximeadir, 'ximea_distCoeffs.npy')

#rootdir = '../../hardware/'
#imperxdir = 'imperx/intrinsics/'
imperxdir = 'Imperx_id2/2020-06-26'
imperx_cameraMatrix=os.path.join(rootdir, imperxdir, 'imperx_cameraMatrix.npy')
imperx_distCoeffs=os.path.join(rootdir, imperxdir, 'imperx_distCoeffs.npy')
# Each of the matrices here are [3, 3] representing the intrinsics projections.
intrinsics = dict(
    ximea=np.load(ximea_cameraMatrix),
    imperx=np.load(imperx_cameraMatrix)
)


# Each array here should be [5] distortion coefficients unless we upgrade
# to a more complicated distortion model.
distortion = dict(
    ximea=np.load(ximea_distCoeffs),
    imperx=np.load(imperx_distCoeffs)
)
