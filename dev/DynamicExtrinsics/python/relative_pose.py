

import numpy as np
import cv2 as cv
import sys, math
from pose_verification import compose_extrinsics, find_relative_pose

# This script is used for estimating the relative pose betwen two charuco board
# based on 2 images taken from the camera (fixed, no movement between images).
# How to use:
# - Change small_charuco to True or False as appropriate based on the subject of the images;
# - Change the paths to the intrinsics for the camera you used if applicable;
# - Run this script with the paths to the 2 images on the command-line as inputs
# Author: Yang Chen (ychen@hrl.com)
# Date:  2020-11-10

small_charuco=False
imperx=True

if small_charuco: # 4x6 in. fiducial
	#squareLength = 0.0127     # These values are for the 4"x6" fiducial that we use for 6DOF estimation
	squareLength = 0.012     # This the charuco board reprinted with exact measurement through PDF download from calib.io
	markerLength = 0.009      # located in "Charuco_Specific/CharucoBoards.py"
	charucoX = 8
	charucoY = 12
else: # TV3
	squareLength = 0.0235     # This the charuco board reprinted with exact measurement through PDF download from calib.io
	markerLength = 0.01665      # located in "Charuco_Specific/CharucoBoards.py"
	charucoX = 40
	charucoY = 20

aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
board = cv.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

if imperx:
	cameraMatrix = np.load('../../../src/Config/CameraIntrinsics/Imperx_id2/2020-09-16/imperx_cameraMatrix.npy')
	distCoeffs = np.load('../../../src/Config/CameraIntrinsics/Imperx_id2/2020-09-16/imperx_distCoeffs.npy')
else: # Ximea
	cameraMatrix = np.load('../../../src/Config/CameraIntrinsics/Ximea_id0/2020-07-15/ximea_cameraMatrix.npy')
	distCoeffs = np.load('../../../src/Config/CameraIntrinsics/Ximea_id0/2020-07-15/ximea_distCoeffs.npy')
#cameraMatrix = np.load('../../../src/Config/CameraIntrinsics/Imperx_id2/2020-06-26/imperx_cameraMatrix.npy')
#distCoeffs = np.load('../../../src/Config/CameraIntrinsics/Imperx_id2/2020-06-26/imperx_distCoeffs.npy')
print('camera matrix:\n', cameraMatrix)
print('camera distCoeffs:\n', distCoeffs)



def charuco_reprojection_error(corners,ids,board,rvec,tvec,cameraMatrix,distCoeffs, image=None):

    #objpoints , imgpoints = cv.aruco.getBoardObjectAndImagePoints(board, corners, ids)
    #reppoints, _ = cv.projectPoints(objpoints, rvec, tvec, cameraMatrix, distCoeffs)

    chessboardCorners = board.chessboardCorners[ ids ]

    reppoints, _ = cv.projectPoints(chessboardCorners, rvec, tvec, cameraMatrix, distCoeffs)
    #rms_error = np.sqrt(np.mean(np.square(np.float64(corners - reppoints))))
    rms_error = np.linalg.norm(corners - reppoints)/math.sqrt(2*len(corners))
    # Draw the reprojected points on the image if given
    if image is not None:
    	for pt in reppoints:
    		pt = (int(pt[0][0]), int(pt[0][1]))
    		#cv.line(image, (100.0, 1.0), (1, 100), (255, 0, 0), thickness=lineThickness)
    		cv.line(image, (pt[0]-10, pt[1]), (pt[0]+10, pt[1]), (0, 0, 200), thickness=1)
    		cv.line(image, (pt[0], pt[1]-10), (pt[0], pt[1]+10), (0, 0, 200), thickness=1)
    return rms_error

def getBoardPose(image):

	params = cv.aruco.DetectorParameters_create()
	if small_charuco:
		params.minMarkerPerimeterRate = 0.01
		params.polygonalApproxAccuracyRate = 0.07
		params.adaptiveThreshWinSizeStep = 1
		params.adaptiveThreshConstant = 2
	markerCorners, ids, rejectedImgPts = cv.aruco.detectMarkers(image, aruco_dict, None, None, params)
	print('Total markers detected:', len(ids))
	
	if len(ids)< 10:
		print('Error: Too few markers detected.')
		return
	# get the checker board corner for use later
	retval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
	print('Total corners interpolated:', len(charucoIds))

	retval, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, (), ())
	reproj_err_rms = charuco_reprojection_error(charucoCorners, charucoIds, board, rvec, tvec, cameraMatrix, distCoeffs, image)
	
	print("reprojection error (RMS): ", reproj_err_rms)

	print('Pose estimation successful?', retval)

	return rvec, tvec

image=None
idx = 0
rvectvec = []

# call this program w/ the paths to 2 Imperx images to find out the relative pose

while len(sys.argv) > 1:
	imagefile = sys.argv[1]

	print('Loading image from: ', imagefile)

	image = cv.imread(imagefile)

	rvec, tvec = getBoardPose(image)

	rvectvec.append((rvec, tvec))
	idx += 1
	sys.argv = sys.argv[1:]

	#win = cv.namedWindow("ChaRuCo Board", cv.WINDOW_GUI_EXPANDED|cv.WINDOW_KEEPRATIO)
	#cv.imshow("ChaRuCo Board", image)
	#cv.waitKey(0)

	if idx >= 2: break  # only need first 2 images


extrinsics1 = compose_extrinsics(rvectvec[0][0],rvectvec[0][1])
extrinsics2 = compose_extrinsics(rvectvec[1][0],rvectvec[1][1])

print('extrinsics 1 = \n', extrinsics1)
print('extrinsics 2 = \n', extrinsics2)
print('\nRelative pose:')
find_relative_pose(extrinsics1, extrinsics2)

