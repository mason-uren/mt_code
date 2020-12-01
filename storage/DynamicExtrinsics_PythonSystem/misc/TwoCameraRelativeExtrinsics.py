import cv2
import glob
import numpy as np
from Charuco_Specific.CharucoBoards import *
from Charuco_Specific.ChArUcoHelpers import *

INDEX = 2

################################
# Define Board Parameters      #
################################
squareLength = boards['TV']['squareLength']
markerLength = boards['TV']['markerLength']
charucoX = boards['TV']['charucoX']
charucoY = boards['TV']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

################################
# Load Intrinsics              #
################################
ximea_intrinsics = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_intrinsics.npy")
ximea_dist = np.load("/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/ximea/intrinsics/ximea_distCoeffs.npy")
imperx_intrinsics = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_intrinsics.npy')
imperx_dist = np.load('/home/sdrad/PycharmProjects/ClosedLoopMetrology/hardware/imperx/intrinsics/imperx_distCoeffs.npy')

print("Ximea Intrinsics")
print(ximea_intrinsics)

print("Ximea Distortion")
print(ximea_dist)
print(' \n ')

print("Imperx Intrinsics")
print(imperx_intrinsics)

print("Imperx Distortion")
print(imperx_dist)
print(' \n\n ')

################################
# Load Ximea images            #
################################
calcorners = []
calids = []

filenames = glob.glob("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Relative_Extrinsics_Photos/extrinsics_photos_PT_location_6/ximea/*")
filenames.sort()

image_list = [cv2.imread(img,0) for img in filenames]

print("Number of Images in Ximea Folder: " + str(len(image_list)))
ximea_shape = image_list[0].shape

image = image_list[INDEX]
print(filenames[INDEX])

# low pass filter maybe
# image = cv2.blur(image, (3, 3))
# image = cv2.resize(image, (7900, 6000))
corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)

if ids is not None:
    ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, image, board)


    retval, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, ximea_intrinsics,
                                                            ximea_dist, useExtrinsicGuess=False)

    charuco_reprojection_error(board,corners,ids,rvecs,tvecs,ximea_intrinsics,ximea_dist)


    rvec_ximea = cv2.Rodrigues(rvecs)
    rvec_ximea = np.array(rvec_ximea[0])
    tvec_ximea = tvecs

    rvec_ximea = cv2.Rodrigues(rvecs)
    rvec_ximea = np.array(rvec_ximea[0])
    tvec_ximea = tvecs

    print("Ximea Rvec: " + str(rvec_ximea))
    print("Ximea Tvec: " + str(tvec_ximea))
    print("\n\n")



################################
# Load Imperx images           #
################################
calcorners = []
calids = []

filenames = glob.glob("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Relative_Extrinsics_Photos/extrinsics_photos_PT_location_6/imperx/*")
filenames.sort()

image_list = [cv2.imread(img) for img in filenames]
print("Number of Images in Imperx Folder: " + str(len(image_list)))
imperx_shape_x , imperx_shape_y, _ = image_list[0].shape

image = image_list[INDEX]
print(filenames[INDEX])

    # image = cv2.resize(image,(5120, 5120))
corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
if ids is not None:
    ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, image, board)

    retval, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(chcorners, chids, board, imperx_intrinsics,
                                                            imperx_dist, useExtrinsicGuess=False)
    charuco_reprojection_error(board,corners,ids,rvecs,tvecs,imperx_intrinsics,imperx_dist,image)

    rvec_imperx = cv2.Rodrigues(rvecs)
    rvec_imperx = np.array(rvec_imperx[0])
    tvec_imperx = tvecs


print("Imperx Rvec: " + str(rvec_imperx))
print("Imperx Tvec: " + str(tvec_imperx))
print("\n\n")

print("Ximea to Imperx")
imperx_extrinsics =  np.hstack((rvec_imperx,tvec_imperx))
imperx_extrinsics = np.vstack((imperx_extrinsics,[[0,0,0,1]]))

ximea_extrinsics =  np.hstack((rvec_ximea,tvec_ximea))
ximea_extrinsics = np.vstack((ximea_extrinsics,[[0,0,0,1]]))


ximea_extrinsics_inverted = np.linalg.inv(ximea_extrinsics)
composed_extrinsics = np.matmul(imperx_extrinsics,ximea_extrinsics_inverted)
print(composed_extrinsics)
# print("Tvec: " + str(T))
print("\n \n")


extrinsics_ximea = np.hstack((rvec_ximea, tvec_ximea))
extrinsics_ximea = np.vstack((extrinsics_ximea,[[0,0,0,1]]))
extrinsics_imperx = np.hstack((rvec_imperx, tvec_imperx))
extrinsics_imperx = np.vstack((extrinsics_imperx,[[0,0,0,1]]))



np.save("imperx",extrinsics_imperx)
np.save("ximea",extrinsics_ximea)
np.save("Ximea_Imperx",composed_extrinsics)