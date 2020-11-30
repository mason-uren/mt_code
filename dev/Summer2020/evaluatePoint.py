# Last Modified: 7/18/2020 Changes made by Aaron
# Aaron added for personal import setup
from setupImports import setup_path
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
import scipy.stats as stats
from dynamicIntrinsicsHelpers import *

# image_list should be the test images
# Input: intrinsicsList, distortionList, focusList, distanceList, image_list

# For each image, apply solvePnP with each intrinsics, distortion pair and
# store the resulting rvec, tvec

# Plot for that image the resulting rvec (converted to roll, pitch, yaw) and
# tvec as function of the focus position, distance and reprojection error

def testPointEstimates(intrinsicsList, distortionList, focusList, distanceList, pointIndex, image_list):
    for i, image in enumerate(image_list):
        print('On image: ' + str(i))
        
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        rotations = []
        translations = []
        errors = []
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            for j in range(len(intrinsicsList)):
                cameraMatrix = intrinsicsList[j]
                distCoeffs = distortionList[j]
                
                retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, cameraMatrix, distCoeffs, None, None, False)
                
                R, _ = cv2.Rodrigues(rvec)
                rotations.append(np.rad2deg(rotationMatrixToEulerAngles(R)))
                translations.append(tvec)
                
                mean_err = charuco_reprojection_error(board, chorners, chids, rvec, tvec, cameraMatrix, distCoeffs)
                errors.append(mean_err)
                
        rotations = np.squeeze(rotations)
        translations = np.squeeze(translations)
        
        names = ['Roll', 'Pitch', 'Yaw']
        for k in range(3):
            plt.figure()
            plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': ' + names[k] + ' [deg]')
            plt.scatter(focusList, rotations[:,k])
            plt.axvline(x = focusList[pointIndex], linestyle='dashed')
            plt.xlabel('Focus Position')
            plt.ylabel(names[k] + ' [deg]')
        
        names = ['X', 'Y', 'Z']
        for k in range(3):
            plt.figure()
            plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': ' + names[k] + ' [mm]')
            plt.scatter(focusList, translations[:,k] * 1e3)
            plt.axvline(x = focusList[pointIndex], linestyle='dashed')
            plt.xlabel('Focus Position')
            plt.ylabel(names[k] + ' [mm]')

        plt.figure()
        plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': Re-projection Error')
        plt.scatter(focusList, errors)
        plt.axvline(x = focusList[pointIndex], linestyle='dashed')
        plt.xlabel('Focus Position')
        plt.ylabel('Re-projection Error')

# See what impact of varying q is on tvec and rvec (leaving rest the same, using the appropriate point estimate for rest)
# Repeat this for u0, v0, k1, k2

# For each image, apply the appropriate default intrinsics and distortion
# Then, while leaving the other components unchanged, swap out one component for the other point estimated values
# and see the result on rvec, tvec

def perturbPointEstimates(intrinsicsList, distortionList, focusList, distanceList, pointIndex, image_list):
    for i, image in enumerate(image_list):
        print('On image: ' + str(i))
        
        parameters = cv2.aruco.DetectorParameters_create()
    
        markerCorners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, None, None, parameters)
        
        paramNames = ['f', 'u0', 'v0', 'k1', 'k2']
        # if we dont find enough points skip
        if (ids is not None and len(ids) > 8):
            ret, chorners, chids = cv2.aruco.interpolateCornersCharuco(markerCorners, ids, image, board)
            
            f = [intrinsicsList[i][0,0] for i in range(len(intrinsicsList))]
            u0 = [intrinsicsList[i][0,2] for i in range(len(intrinsicsList))]
            v0 = [intrinsicsList[i][1,2] for i in range(len(intrinsicsList))]
            k1 = [distortionList[i][0,0] for i in range(len(distortionList))]
            k2 = [distortionList[i][0,1] for i in range(len(distortionList))]
    
            defaultIntrinsics = intrinsicsList[pointIndex]
            defaultDist = distortionList[pointIndex]
            
            for j, paramList in enumerate([f, u0, v0, k1, k2]):
                rotations = []
                translations = []
                errors = []
                for param in paramList:
                    cameraMatrix = np.copy(defaultIntrinsics)
                    distCoeffs = np.copy(defaultDist)
                    if j == 0:
                        cameraMatrix[0,0] = param
                        cameraMatrix[1,1] = param
                    elif j == 1:
                        cameraMatrix[0,2] = param
                    elif j == 2:
                        cameraMatrix[1,2] = param
                    elif j == 3:
                        distCoeffs[0,0] = param
                    elif j == 4:
                        distCoeffs[0,1] = param
                
                    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(chorners, chids, board, cameraMatrix, distCoeffs, None, None, False)
                    
                    R, _ = cv2.Rodrigues(rvec)
                    rotations.append(np.rad2deg(rotationMatrixToEulerAngles(R)))
                    translations.append(tvec)
                    
                    mean_err = charuco_reprojection_error(board, chorners, chids, rvec, tvec, cameraMatrix, distCoeffs)
                    errors.append(mean_err)
                
                # Plot the rotations and translations against the changing parameter, put vertical line for default
                rotations = np.squeeze(rotations)
                translations = np.squeeze(translations)
            
                names = ['Roll', 'Pitch', 'Yaw']
                for k in range(3):
                    plt.figure()
                    plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': ' + names[k] + ' [deg]')
                    plt.scatter(paramList, rotations[:,k])
                    plt.axvline(x = paramList[pointIndex], linestyle='dashed')
                    plt.xlabel(paramNames[j])
                    plt.ylabel(names[k] + ' [deg]')
                        
                names = ['X', 'Y', 'Z']
                for k in range(3):
                    plt.figure()
                    plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': ' + names[k] + ' [m]')
                    plt.scatter(paramList, translations[:,k])
                    plt.axvline(x = paramList[pointIndex], linestyle='dashed')
                    plt.xlabel(paramNames[j])
                    plt.ylabel(names[k] + ' [m]')
        
                plt.figure()
                plt.title('Focus = ' + str(focusList[pointIndex]) + ' Image ' + str(i) + ': Re-projection Error')
                plt.scatter(paramList, errors)
                plt.axvline(x = paramList[pointIndex], linestyle='dashed')
                plt.xlabel(paramNames[j])
                plt.ylabel('Re-projection Error')
            
##########################
# Charuco Board Consts   #
##########################

squareLength = boards['TV_3']['squareLength']
markerLength = boards['TV_3']['markerLength']
# In the images, seems like board is actually rotated so that have 12 across 
# by 8 down 
charucoX = boards['TV_3']['charucoX']
charucoY = boards['TV_3']['charucoY']

##########################
# Graphng Setup          #
##########################
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)

# Aaron added to verify that board is indeed what are seeing in the images
dispBoard = board.draw((500, 500))
#cv2.imshow('board', dispBoard)
#cv2.imshow('board', cv2.rotate(dispBoard, cv2.ROTATE_90_CLOCKWISE))
#cv2.waitKey(0)
    
##########################
# Load Images from Path  #
##########################
import glob

# Need to separate the imperx and ximea
cameraName = 'ximea' # or imperx

numRounds = 6

focusList = []

distanceList = [181, 125, 135.5, 147.5, 159, 169.5]

intrinsicsList = []
distortionList = []
paramsList = []
rounds = []
overallImages = []

graph = True

# Ignore outlier rounds
for AFround in [2, 3, 5, 1]: # range(numRounds + 1):
    #foldername = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined/AFround' + str(AFround)
    foldername = 'C:/Users/aofeldman/Desktop/testCollection8-31/AFround' + str(AFround)
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
    
        cameraMatrix = np.load(foldername + '/' + cameraName + '_intrinsics.npy')
        distCoeffs = np.load(foldername + '/' + cameraName + '_distCoeffs.npy')
    
        intrinsicsList.append(cameraMatrix)
        distortionList.append(distCoeffs)
        rounds.append(AFround)
        focusList.append(searchForFocus(foldername + '/' + 'AF' + str(AFround) + '.txt'))
        overallImages.append(image_list)
            
# TODO: Reserve a test set because right now, the point estimate used this image in calibration. Unfair advantage
for i, image_list in enumerate(overallImages):
    # For now, do on a single image
    image_list = [image_list[0]]
    testPointEstimates(intrinsicsList, distortionList, focusList, distanceList, i, image_list)
    perturbPointEstimates(intrinsicsList, distortionList, focusList, distanceList, i, image_list)