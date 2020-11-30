# Last Modified: 6/30/2020 Changes made by Aaron
# Used to see what parameters work well for detecting the L-beam in the imperx
# images for a dataset

# Aaron added for personal import setup
from setupImports import setup_path
setup_path()

from hardware.ximea.Driver.client.ximea_client import *
from Charuco_Specific.ChArUcoHelpers import *
from Charuco_Specific.CharucoBoards import *
import numpy as np
import math
import time
import shutil
import glob
import pdb
import os
import matplotlib.pyplot as plt

##########################
# Charuco Board Consts   #
##########################

squareLength = boards['Fiducial']['squareLength']
markerLength = boards['Fiducial']['markerLength']
# In the images, seems like board is actually rotated so that have 12 across 
# by 8 down 
charucoX = boards['Fiducial']['charucoX']
charucoY = boards['Fiducial']['charucoY']

##########################
# Graphng Setup          #
##########################
start_time = time.time()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', cv2.rotate(dispBoard, cv2.ROTATE_90_CLOCKWISE))
#cv2.waitKey(0)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
DISP_IMG = False

# Need to separate the imperx and ximea
cameraName = 'ximea' # or Ximea

#filenames = glob.glob('C:/Users/aofeldman/Desktop/L-Beam_tests/'+cameraName+'/*.jpg')
filenames = glob.glob('C:/Users/aofeldman/Desktop/testCollection7-21Refined/AFround10/L-Beam/' + cameraName + '*.tif')

filenames.sort()

# Aaron added in reading in as grayscale
image_list = [cv2.imread(img, 0) for img in filenames]

print(image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

# for i, image in enumerate(image_list):
#     # Subsampling is True now
#     dist = np.load('imperx_distCoeffs.npy')
#     intrinsics = np.load('imperx_intrinsics.npy')
#     # Set True for cropping
#     extrinsics_4x4,reprojection_error,chorners = estimate_Pose_Charucoboard_Imperx(image,board,intrinsics,dist,aruco_dict,False)
#     pdb.set_trace()

def detectionRun(image_list, aruco_dict, board, parameters=cv2.aruco.DetectorParameters_create(), saveImages=False):
    numMarkersDetected = []
    for i, image in enumerate(image_list):
        image = cv2.blur(image,(3,3))
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        #cropsize = (image.shape[1] // 4, image.shape[0] // 4)
        #croppedIm, _ = search_by_crop_for_charucoBoard(image, aruco_dict, cropsize)
        #estimate_Pose_Charucoboard_Imperx(image,board,intrinsics,dist,aruco_dict, True)
        numMarkersDetected.append(len(corners))
        
    #     if ids is not None:
    #         ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board)
    
    #         calcorners.append(chcorners)
    #         calids.append(chids)
    #         ids = np.reshape(ids, (ids.shape[0],))
    #         numCornersDetected.append(len(chcorners))
    #     else:
    #         calcorners.append([])
    #         calids.append([])
    #         numCornersDetected.append(0)
        
    #     print('Number of corners detected: ', numCornersDetected[-1])

    #     disp = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    #     if ids is not None:
    #         cv2.aruco.drawDetectedCornersCharuco(disp, chcorners, chids, (0, 0, 255))
    #     #disp = cv2.resize(disp,(1000, 1000))
    #     #cv2.imshow("window", disp)
    #     #cv2.waitKey(1)
    #     if saveImages:
    #         cv2.imwrite('C:/Users/aofeldman/Desktop/testingParameters/'+str(i) + '.jpg', disp)
    # #cv2.destroyAllWindows()
    
    #return numCornersDetected, calids, calcorners
    return numMarkersDetected

# 2*8*12/2 = 96 total markers in image


# Decent parameters: 
parameters = cv2.aruco.DetectorParameters_create()
parameters.minMarkerPerimeterRate = 0.01
parameters.polygonalApproxAccuracyRate = 0.07
parameters.adaptiveThreshWinSizeStep = 1
parameters.adaptiveThreshConstant = 2

saveImages = True
numCornersDetected = []
for i, image in enumerate(image_list):
    image = cv2.blur(image,(3,3))
    
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
    if ids is not None:
        ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board)

        calcorners.append(chcorners)
        calids.append(chids)
        ids = np.reshape(ids, (ids.shape[0],))
        numCornersDetected.append(len(chcorners))
    else:
        calcorners.append([])
        calids.append([])
        numCornersDetected.append(0)
    
    print('Number of corners detected: ', numCornersDetected[-1])

    disp = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    if ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(disp, chcorners, chids, (0, 0, 255))
    #disp = cv2.resize(disp,(1000, 1000))
    #cv2.imshow("window", disp)
    #cv2.waitKey(1)
    if saveImages:
        cv2.imwrite('C:/Users/aofeldman/Desktop/testingParameters/'+str(i) + '.jpg', disp)
#cv2.destroyAllWindows()





# Specify minMarkerPerimeterRate
perimeterList = [0.005*i for i in range(1, 7)]

# Specify polygonalApproxAccuracyRate
polyList = [0.01*i for i in range(5, 13)]

# minCornerDistanceRate: Reduce from 0.05 to loosen
cornerList = [0.005*i for i in range(1,11)]

# Reduce from 3 to loosen, adaptiveThreshWinSizeMin
winSizeMinList = [1, 2, 3]

# Reduce from 10, adaptiveThreshWinSizeStep
winStepList = [i for i in range(1, 11)]

# Default 7, adaptiveThreshConstant
winConstList = [i for i in range(1, 15)]

# Turn off inline plotting
%matplotlib qt5

if True:
    # Set to 0.01 to saturate
    plt.figure()
    plt.title('Changing minMarkerPerimeterRate')
    for val in perimeterList:
        print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = val
        numMarkersDetected = detectionRun(image_list, aruco_dict, board, parameters)
        plt.plot(range(len(numMarkersDetected)), numMarkersDetected)
    plt.legend(perimeterList)
    plt.ylabel('Number of Corners Detected')
    plt.xlabel('Image Number')
    plt.show()

# Minimal impact
if True:
    plt.figure()
    plt.title('Changing polygonalApproxAccuracyRate')
    for val in polyList:
        print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = 0.01
        parameters.polygonalApproxAccuracyRate = val
        numMarkersDetected = detectionRun(image_list, aruco_dict, board, parameters)
        plt.plot(range(len(numMarkersDetected)), numMarkersDetected)
    plt.legend(polyList)
    plt.ylabel('Number of Corners Detected')
    plt.xlabel('Image Number')
    plt.show()

# Has no impact
if False:
    plt.figure()
    plt.title('Changing minCornerDistanceRate')
    for val in cornerList:
        print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = 0.01
        parameters.minCornerDistanceRate = val
        numMarkersDetected = detectionRun(image_list, aruco_dict, board, parameters)
        plt.plot(range(len(numMarkersDetected)), numMarkersDetected)
    plt.legend(cornerList)
    plt.ylabel('Number of Corners Detected')
    plt.xlabel('Image Number')
    plt.show()

if False:
    plt.figure()
    plt.title('Changing window thresholding step')
    for val in winStepList:
        print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = 0.01
        parameters.polygonalApproxAccuracyRate = 0.07
        parameters.adaptiveThreshWinSizeStep = val
        numMarkersDetected = detectionRun(image_list, aruco_dict, board, parameters)
        plt.plot(range(len(numMarkersDetected)), numMarkersDetected)
    plt.legend(winStepList)
    plt.ylabel('Number of Corners Detected')
    plt.xlabel('Image Number')
    plt.show()

if True:
    plt.figure()
    plt.title('Changing window thresholding const')
    for val in winConstList:
        print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = 0.01
        parameters.polygonalApproxAccuracyRate = 0.07
        parameters.adaptiveThreshWinSizeStep = 1
        parameters.adaptiveThreshConstant = val
        numMarkersDetected = detectionRun(image_list, aruco_dict, board, parameters)
        plt.plot(range(len(numMarkersDetected)), numMarkersDetected)
    plt.legend(winConstList)
    plt.ylabel('Number of Corners Detected')
    plt.xlabel('Image Number')
    plt.show()