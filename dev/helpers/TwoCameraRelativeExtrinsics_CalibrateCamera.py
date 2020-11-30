import cv2
import glob
import numpy as np
from Charuco_Specific.CharucoBoards import *

squareLength = boards['TV']['squareLength']
markerLength = boards['TV']['markerLength']
charucoX = boards['TV']['charucoX']
charucoY = boards['TV']['charucoY']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

################################
# Load Ximea images            #
################################
calcorners = []
calids = []

filenames = glob.glob("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Relative_Extrinsics_Photos/extrinsics_photos_PT_location_7/ximea/*")
filenames.sort()
image_list = [cv2.imread(img,0) for img in filenames]

print("Number of Images in Ximea Folder: " + str(len(image_list)))

ximea_shape = image_list[0].shape

for image in image_list:
    # image = cv2.resize(image,(7900, 6000))
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))

    # cv2.aruco.drawDetectedCornersCharuco(image, chcorners, chids, (0, 255, 0))
    # image = cv2.resize(image,(1000, 1000))
    # cv2.imshow("window", image)
    # cv2.waitKey(1)

rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    calcorners, calids, board, ximea_shape, None, None)

rvec_ximea = cv2.Rodrigues(rvecs[0])
rvec_ximea = np.array(rvec_ximea[0])
tvec_ximea = tvecs[0]

print("Ximea Reprojection Error: " + str(rms))
print("Ximea Rvec: " + str(rvec_ximea))
print("Ximea Tvec: " + str(tvec_ximea))
print("\n\n")

################################
# Load Imperx images           #
################################
calcorners = []
calids = []

filenames = glob.glob("/home/sdrad/PycharmProjects/ClosedLoopMetrology/Experiment_Results/Relative_Extrinsics_Photos/extrinsics_photos_PT_location_7/imperx/*")
filenames.sort()
image_list = [cv2.imread(img) for img in filenames]
print("Number of Images in Imperx Folder: " + str(len(image_list)))
imperx_shape = image_list[0].shape[0:2]
print(imperx_shape)
for image in image_list:
    # image = cv2.resize(image,(5120, 5120))
    corners, ids, rejectedImgPts = cv2.aruco.detectMarkers(image, aruco_dict)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))

        # cv2.aruco.drawDetectedCornersCharuco(image, chcorners, chids, (0, 255, 0))
        # image = cv2.resize(image,(1000, 1000))
        # cv2.imshow("window", image)
        # cv2.waitKey(1)

rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    calcorners, calids, board, imperx_shape, None, None)

rvec_imperx = cv2.Rodrigues(rvecs[0])
rvec_imperx = np.array(rvec_imperx[0])
tvec_imperx = tvecs[0]


imperx_extrinsics =  np.hstack((rvec_imperx,tvec_imperx))
imperx_extrinsics = np.vstack((imperx_extrinsics,[[0,0,0,1]]))

ximea_extrinsics =  np.hstack((rvec_ximea,tvec_ximea))
ximea_extrinsics = np.vstack((ximea_extrinsics,[[0,0,0,1]]))


ximea_extrinsics_inverted = np.linalg.inv(ximea_extrinsics)
composed_extrinsics = np.matmul(imperx_extrinsics,ximea_extrinsics_inverted)

print("Imperx Reprojection Error: " + str(rms))
print("Imperx Reprojection Error: " + str(rms))
print("Imperx Rvec: " + str(rvec_imperx))
print("Imperx Tvec: " + str(tvec_imperx))
print("\n\n")

print("Ximea to Imperx")
print(composed_extrinsics)
print("\n \n")

np.save("imperx_one",imperx_extrinsics)
np.save("ximea_one",ximea_extrinsics)
np.save("Ximea_Imperx_one",composed_extrinsics)