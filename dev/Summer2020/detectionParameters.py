# Last Modified: 6/23/2020 Changes made by Aaron
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

# Aaron Changed to TV_4 was TV_3, put back to TV_3 since used big monitor again
squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

##########################
# Graphng Setup          #
##########################
start_time = time.time()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', dispBoard)
#cv2.waitKey(0)

calcorners = []
calids = []

##########################
# Load Images from Path  #
##########################
import glob

DISP_IMG = False

# Need to separate the imperx and ximea
cameraName = 'imperx' # or ximea 

# Aaron modified the path
filenames = glob.glob('C:/Users/aofeldman/Desktop/'+cameraName+'Data/*.tif')

# Aaron changed so would respect ordering of images in the data set
revFilenames = []
for i in range(len(filenames)):
    revFilenames.append('C:/Users/aofeldman/Desktop/'+cameraName+'Data/' +cameraName + str(i) + '.tif')
    
filenames = revFilenames

# Aaron added in reading in as grayscale
image_list = [cv2.imread(img, 0) for img in filenames]

print(image_list[0].shape[0])

print("Number of Images in Folder: " + str(len(image_list)))

def detectionRun(image_list, aruco_dict, board, parameters=cv2.aruco.DetectorParameters_create(), saveImages=False):
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
    
    return numCornersDetected, calids, calcorners

# Parameters to investigate:
# minMarkerPerimeterRate: Reduce from default of 0.03 to loosen
# polygonalApproxAccuracyRate: Increase from default of 0.05 to loosen
# minCornerDistanceRate: Reduce from 0.05 to loosen

# Specify minMarkerPerimeterRate
perimeterList = [0.005*i for i in range(1, 7)]

# Turn off inline plotting
%matplotlib qt

plt.figure()
plt.title('Changing minMarkerPerimeterRate')
for val in perimeterList:
    print("--> STARTED A NEW PARAMETER VALUE: " + str(val))
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.minMarkerPerimeterRate = val
    numCornersDetected, _, _ = detectionRun(image_list, aruco_dict, board, parameters)
    plt.plot(range(len(numCornersDetected)), numCornersDetected)
plt.legend(perimeterList)
plt.ylabel('Number of Corners Detected')
plt.xlabel('Image Number')
plt.show()

parameters = cv2.aruco.DetectorParameters_create()
parameters.minMarkerPerimeterRate = 0.025
detectionRun(image_list, aruco_dict, board, parameters, True)