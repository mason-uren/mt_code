import numpy as np
import cv2
from h5Helpers import *
from ChArUcoHelpers import *
import h5py

imperx = cv2.imread("imperx.jpg")
ximea = cv2.imread("ximea.jpg")

imperx_intrinsics = np.load("imperx_intrinsics.npy")
imperx_dist = np.load("imperx_distCoeffs.npy")

print(imperx_intrinsics)

ximea_intrinsics = np.load("ximea_intrinsics.npy")
ximea_dist = np.load("ximea_distCoeffs.npy")

print(ximea_intrinsics)

squareLength = 0.083   #in meters
markerLength = 0.064   #in meters
charuco_board_x = 12
charuco_board_y = 12

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(charuco_board_x, charuco_board_y, squareLength, markerLength, dictionary)

corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(imperx, dictionary)
retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids,board,imperx_intrinsics,imperx_dist)

print("Imperx Rvec: " + str(rvec) + "\n")
print("Imperx Tvec: " + str(tvec) + "\n\n")

corners1, ids1, rejectedImgPts = cv2.aruco.detectMarkers(ximea, dictionary)
retval, rvec1, tvec1  = cv2.aruco.estimatePoseBoard(corners1, ids1, board, ximea_intrinsics, ximea_dist)
print("Ximea Rvec: " + str(rvec1) + "\n")
print("Ximea Tvec: " + str(tvec1) + "\n\n")

f = h5py.File('imperx_intrinsics', 'w')
f['intrinsics'] = imperx_intrinsics
f['dist'] = imperx_dist

f = h5py.File('imperx_extrinsics', 'w')
f['rvec'] = rvec
f['tvec'] = tvec

f = h5py.File('ximea_intrinsics', 'w')
f['intrinsics'] = ximea_intrinsics
f['dist'] = ximea_dist

f = h5py.File('ximea_extrinsics', 'w')
f['rvec'] = rvec1
f['tvec'] = tvec1

info = cv2.composeRT(rvec1,tvec1,rvec,tvec)

f = h5py.File('extrinsics_Ximea_to_Imperx', 'w')
f['rvec'] = np.array(info[0])
f['tvec'] = np.array(info[1])

rvec = cv2.Rodrigues(info[0])
extrinsics = np.hstack((rvec[0],np.array(info[1])))
f['composed_ext'] = extrinsics

print(extrinsics)

ximea = cv2.undistort(ximea,ximea_intrinsics,ximea_dist)
imperx = cv2.undistort(imperx,imperx_intrinsics,imperx_dist)

img = cv2.undistort(imperx,imperx_intrinsics,imperx_dist)
imperx = [imperx]
ximea = [ximea]
img1_dict, img2_dict = get_common_charuco_ids(imperx, ximea, board, dictionary)

print(img1_dict)
print(img2_dict)
