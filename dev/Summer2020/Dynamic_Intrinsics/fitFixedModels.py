# Last Modified: 7/18/2020 Changes made by Aaron
# Aaron added for personal import setup
from setupImports2 import setup_path
setup_path()

# from hardware.ximea.Driver.client.ximea_client import *
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
import pandas as pd 

plt.close('all')

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

# Aaron commented out because don't actually care about running camera
#cam = ximea_recieve_camera()
#cam.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
dispBoard = board.draw((1000, 1000))
#cv2.imshow('board', dispBoard)
#cv2.waitKey(0)

writePath = 'C:/Users/aofeldman/Desktop/pointResults.xlsx'

##########################
# Helper Functions       #
##########################

# Compute mean reprojection error over a dataset given the calibration results
# Assumes blurred already
def error_for_dataset(image_list, intrinsics, dist, cameraName='imperx'):
    image_errors = []
    for i, image in enumerate(image_list):
    
        # Aaron testing out different parameter options
        parameters = cv2.aruco.DetectorParameters_create()
        
        # Empirically observed that for the dataset slightly reducing this value
        # from the default results in significantly better detection for the 
        # imperx
        if cameraName == 'imperx':
            parameters.minMarkerPerimeterRate = 0.025
        
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, intrinsics, dist, None, None, useExtrinsicGuess=False)
            # Compute the reprojection error
            R, _ = cv2.Rodrigues(rvec)
            mean_error = charuco_reprojection_error(board,chorners,chids,R,tvec,intrinsics,dist)
            image_errors.append(mean_error)
        else:
            print("Skipping: Failed to find charuco board in image " + str(i))
    return image_errors, np.mean(image_errors)

def calibrateCamera(board, aruco_dict, image_list, cameraName='imperx'):
    calcorners = []
    calids = []
    
    for i, image in enumerate(image_list):
        parameters = cv2.aruco.DetectorParameters_create()
        if cameraName == 'imperx':
            parameters.minMarkerPerimeterRate = 0.025
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        if (ids is not None and len(ids) > 8):
            ret, chcorners, chids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, image, board)
    
            calcorners.append(chcorners)
            calids.append(chids)
            ids = np.reshape(ids, (ids.shape[0],))
            #print(len(chcorners))
        
    # Proceed with default model
    rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        calcorners, calids, board, (image_list[0].shape[1],image_list[0].shape[0]), None, None)
    
    return rms, cameraMatrix, distCoeffs, rvecs, tvecs


def searchForFocus(filename):
    with open(filename, 'r') as file:
        data = file.read()
        substring = 'Best focus position is'
        location = data.find(substring)
        croppedStr = data[location+len(substring):]
        # Split at spaces and find first number
        for word in croppedStr.split(): # Split at spaces
            # Delete any commas    
            word = word.replace(',', "")
            try:
                focusPosition = int(word)
                return focusPosition
            except ValueError:
                continue
    file.close()

##########################
# Load Images from Path  #
##########################
import glob

# Need to separate the imperx and ximea
cameraName = 'ximea' # or imperx

numRounds = 12

focusList = []
intrinsicsList = []
distortionList = []
meanErrList = []
rmsErrList = []

rounds = []

aspect1 = True
graph = True
saveIntrinsics = False
savePoint = True

for AFround in range(numRounds + 1):
    #foldername = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined/AFround' + str(AFround)
    foldername = 'C:/Users/aofeldman/Desktop/testCollection9-11/AFround' + str(AFround)
    # Load in all images for the given focus setting
    filenames = glob.glob(foldername + '/*.tif')
    
    imageNumbers = [int(filenames[i].replace(foldername+'\\' + cameraName, "").replace('.tif', "")) for i in range(len(filenames))]
    filenames = [filenames[int(imageNumber)] for imageNumber in np.argsort(imageNumbers)]
    imageNumbers = np.sort(imageNumbers)
    
    print('imageNumbers\n', imageNumbers)
    
    # Skip over AF rounds where did not collect any images
    if filenames:
        # Read in and mildly blur images
        image_list = [cv2.blur(cv2.imread(img, 0), (3,3)) for img in filenames]
    
        print('AFround = ' + str(AFround))
        print('Size of Images Collected = ' + str(image_list[0].shape))
        print("Number of Images in Folder: " + str(len(image_list)))
    
        rms, cameraMatrix, distCoeffs, rvecs, tvecs = calibrateCamera(board, aruco_dict, image_list, cameraName)
        image_errors, mean_error = error_for_dataset(image_list, cameraMatrix, distCoeffs, cameraName)
        
        intrinsicsList.append(cameraMatrix)
        distortionList.append(distCoeffs)
        rounds.append(AFround)
        focusList.append(searchForFocus(foldername + '/' + 'AF' + str(AFround) + '.txt'))
        
        plt.figure()
        plt.title('AFround'+str(AFround) + ' image errors')
        plt.plot(imageNumbers, image_errors)
        print('STD, max, min, mean', np.std(image_errors), np.max(image_errors), np.min(image_errors), np.mean(image_errors))
        print('Mean error for round = ' + str(mean_error))
        print('RMS error for round = ' + str(rms))
        meanErrList.append(mean_error)
        rmsErrList.append(rms)
        
        if saveIntrinsics:
            np.save(foldername + '/' + cameraName + '_distCoeffs', distCoeffs)
            np.save(foldername + '/' + cameraName  + '_intrinsics', cameraMatrix)
            np.savetxt(foldername + '/' + cameraName  + '_distCoeffs.txt', distCoeffs)
            np.savetxt(foldername + '/' + cameraName  + '_intrinsics.txt', cameraMatrix)
            np.save(foldername + '/' + cameraName + '_rvecs', rvecs)
            np.save(foldername + '/' + cameraName + '_tvecs', tvecs)

if aspect1:
    f = [camMatrix[0,0] for camMatrix in intrinsicsList]
else:
    fx = [camMatrix[0,0] for camMatrix in intrinsicsList]
    fy = [camMatrix[1,1] for camMatrix in intrinsicsList]
u0 = [camMatrix[0,2] for camMatrix in intrinsicsList]
v0 = [camMatrix[1,2] for camMatrix in intrinsicsList]
k1 = [distCoeffs[0,0] for distCoeffs in distortionList]
k2 = [distCoeffs[0,1] for distCoeffs in distortionList]
p1 = [distCoeffs[0,2] for distCoeffs in distortionList]
p2 = [distCoeffs[0,3] for distCoeffs in distortionList]
k3 = [distCoeffs[0,4] for distCoeffs in distortionList]

if graph:
    if aspect1:
        parameterNames = ['f', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [f, u0, v0, k1, k2, p1, p2, k3]
    else:    
        parameterNames = ['fx', 'fy', 'u0', 'v0', 'k1', 'k2', 'p1', 'p2', 'k3']
        parameterSet = [fx, fy, u0, v0, k1, k2, p1, p2, k3]
   
    for i, parameter in enumerate(parameterSet):
        plt.figure()
        plt.title('Reference calibration results for ' + str(parameterNames[i]))
        plt.scatter(focusList, parameter)
        plt.xlabel('Focus Position')
        plt.ylabel(parameterNames[i])
        
if savePoint:
    # Given the focusList, intrinsicsList , distortionList, meanErrList, rmsErrList
    # Construct table
    
    order = np.argsort(focusList)
    
    # Use all the points
    inliers = np.array(range(len(order)), dtype=np.int64)[order]
    
    orderedFocus = np.array(focusList)[inliers]
    
    indexNames = ['Round ' + str(i + 1) + ' (closest)' * (i == 0) + '(farthest)' * (i == len(orderedFocus)-1) for i in range(len(orderedFocus))]
    columnNames = ['Focus Motor Position', 'u0', 'v0', 'f', 'k1', 'k2', 'Mean Re-projection Error', 'RMS Re-projection Error']
    
    data = np.vstack([focusList, u0, v0, f, k1, k2, meanErrList, rmsErrList]).T[inliers]
    
    result =  pd.DataFrame(data, index=indexNames, columns=columnNames)
    
    result.to_excel(writePath)