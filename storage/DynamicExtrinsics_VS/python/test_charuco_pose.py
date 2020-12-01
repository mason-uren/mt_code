
''' This is a test program to do pose estimation of a ChaRuCo board.
    Test results can be compared with C++ implementation numerically
    on the same image from Ximea camera.
    The ChaRuCo board we use is a 6-inch by 4-inch board with 12 by 8 squares.
 
    To run this code:

    python test_charuco.py  [<Ximea camera image of the ChaRuCo board>]
	
	This code requires OpenCV + contrib. Tested with v3.4.7 in Linux, and
	v3.4.8 & 4.1.2 on MacOS Sierra (via conda, python 3.7.6).

    Also see results_comparison.txt.

'''
import numpy as np
import cv2 as cv
import sys, math

#Determine the method for pose estimation:
use_charuco_marker = False # True: use aruco markers to estimate pose; less accurate
                          #False: then select one of the choices below
use_charuco_corners = True # True: use charuco corners (implicitly) w/ estimatePoseCharucoBoard()
                          # False: use charuco corners w/ solvePnP()

squareLength = 0.0127     # These values are for the 4"x6" fiducial that we use for 6DOF estimation
markerLength = 0.009      # located in "Charuco_Specific/CharucoBoards.py"
charucoX = 8
charucoY = 12

aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
board = cv.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

cameraMatrix = np.load('../../../dev/hardware/Ximea/intrinsics/ximea_intrinsics.npy')
distCoeffs = np.load('../../../dev/hardware/Ximea/intrinsics/ximea_distCoeffs.npy')
#print('camera intrinsics:\n', cameraMatrix)
#print('camera distCoeffs:\n', distCoeffs)

lineThickness=5
# The model loaded here is used only as an initial guess
#model_path = "../../../dev/DynamicExtrinsics/python/cad_models/2019-09-04-cad-model.json"
#model = PanTiltModel.from_json(model_path)


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

	markerCorners, ids, rejectedImgPts = cv.aruco.detectMarkers(image, aruco_dict,)
	print('Total markers detected:', len(ids))
	
	if len(ids)< 10:
		print('Error: Too few markers detected.')
		return
	# get the checker board corner for use later
	retval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
	print('Total corners interpolated:', len(charucoIds))

	if use_charuco_marker: # use markers for pose estimation; accuracy is not that good.
		cv.aruco.drawDetectedMarkers(image, markerCorners, ids)
		retval, rvec, tvec = cv.aruco.estimatePoseBoard(markerCorners, ids, board, cameraMatrix, distCoeffs,(),())
		print("# markers used for pose estimation: ", retval)
	else: # use checker board corners for pose estimation
		cv.aruco.drawDetectedCornersCharuco(image, charucoCorners, charucoIds,)
		if use_charuco_corners: # use aruco function to estimate pose
			retval, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, (), ())
		else: # do it ourselves, equivalent to calling cv.aruco.estimatePoseCharucoBoard()
			#objPoints, imgPoints = cv.aruco.getBoardObjectAndImagePoints(board, charucoCorners, charucoIds)
			objPoints = board.chessboardCorners[charucoIds]
			retval, rvec, tvec = cv.solvePnP(objPoints, charucoCorners, cameraMatrix, distCoeffs,)
			# There is/are bugs in OpenCV 4.1.2, work-around that didn't actually work: https://github.com/opencv/opencv/issues/16049 
			#reproj_err_rms = np.array([],dtype=np.float64)
			#retval, rvec, tvec= cv.solvePnPGeneric(objPoints, charucoCorners, cameraMatrix, distCoeffs,
			#													  reprojectionError = reproj_err_rms)

	reproj_err_rms = charuco_reprojection_error(charucoCorners, charucoIds, board, rvec, tvec, cameraMatrix, distCoeffs, image)
	print("reprojection error (RMS): ", reproj_err_rms)

	print('Pose estimation successful?', retval)

	#image = cv.aruco.drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, 0.05)
	cv.drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, 0.05, lineThickness)
	print("rvec (deg) =", [ r[0]/3.141592653589793*180 for r in rvec])
	print("tvec = ", [t[0] for t in tvec])

imagefile2 = '../resources/BoardImages/XIMEA [ 0](pose=2.5,5).jpg' # this is on Yang's MacBook, adapt for your machine
image=None

while len(sys.argv) > 1:
	imagefile = sys.argv[1]

	print('Loading image from: ', imagefile)

	image = cv.imread(imagefile)

	getBoardPose(image)

	sys.argv = sys.argv[1:]

if image is None: # load & process the default image
	print('Loading image from: ', imagefile2)
	image = cv.imread(imagefile2)
	getBoardPose(image)

win = cv.namedWindow("ChaRuCo Board", cv.WINDOW_GUI_EXPANDED|cv.WINDOW_KEEPRATIO)

cv.imshow("ChaRuCo Board", image)

cv.waitKey(0)


"""
Results using markers for pose estimation:
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,2.5).jpg
rvec (deg) = [124.67643042104204, 133.67128603892078, 2.4568628073783882]

Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,5).jpg
rvec (deg) = [-119.86952597230864, -128.45052777468712, -5.173312110072999]

Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,8).jpg
rvec (deg) = [-122.90553638689858, -131.86594489449516, 4.891704857268457]

Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-5,5).jpg
rvec (deg) = [123.65774897273714, 131.99717604687308, 8.841161616019226]

Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=2.5,5).jpg
rvec (deg) = [-121.81651825977299, -132.0909509926089, 13.788828818970282]
========================================================================
Results using corners for pose estimation:

python test_charuco.py ../resources/BoardImages/XIMEA*.jpg
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,2.5).jpg
Total markers detected: 46
Total corners interpolated: 73
rvec (deg) = [-120.79001422904348, -129.4893179199931, -2.2780971559154874]
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,5).jpg
Total markers detected: 46
Total corners interpolated: 73
rvec (deg) = [-119.75229138684239, -128.31748446498827, -5.157804477616311]
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-1,8).jpg
Total markers detected: 46
Total corners interpolated: 73
rvec (deg) = [-118.74256082392492, -127.11028332542527, -8.462756504317223]
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=-5,5).jpg
Total markers detected: 46
Total corners interpolated: 73
rvec (deg) = [123.63927238245245, 131.99185934440948, 8.66383136910729]
Loading image from:  ../resources/BoardImages/XIMEA [ 0](pose=2.5,5).jpg
Total markers detected: 46
Total corners interpolated: 73
rvec (deg) = [-117.94993887426057, -126.73295058226218, -2.2448697068416386]
"""

