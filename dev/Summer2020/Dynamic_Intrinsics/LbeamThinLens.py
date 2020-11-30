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
import scipy.optimize
import itertools
from dynamicIntrinsicsHelpers import *

if __name__ == '__main__':
    
    ##########################
    # Charuco Board Consts   #
    ##########################
    
    squareLength = boards['NewFiducial']['squareLength']
    markerLength = boards['NewFiducial']['markerLength']
    # In the images, seems like board is actually rotated so that have 12 across 
    # by 8 down 
    charucoX = boards['NewFiducial']['charucoX']
    charucoY = boards['NewFiducial']['charucoY']
    
    ##########################
    # Graphng Setup          #
    ##########################
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
    
    dispBoard = board.draw((500, 500))
    #cv2.imshow('board', dispBoard)
    #cv2.imshow('board', cv2.rotate(dispBoard, cv2.ROTATE_90_CLOCKWISE))
    #cv2.waitKey(0)
    
    ##########################
    # Load Images from Path  #
    ##########################    
    f = 200 * 1e-3 / 1.0372608996079156
    
    cameraName = 'ximea'
            
    sensor_size = np.array([27.6, 36.4]) * 1e-3
    
    imageShape = (6004, 7920)
    
    pixel2world = sensor_size[0] / imageShape[0]
    
    path = 'C:/Users/aofeldman/Desktop/testCollection8-31'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    numRounds = 6
    trainingRounds = set(range(numRounds+1))
    testRounds = []
    AFround = 3 # Put to -1 to use point estimate at median focus
    
    if AFround == -1:
        
        intrinsicsList, distortionList, focusList, roundsList = loadFixedModels(path, subfolderPrefixes, relDistName, relIntrinsicsName, relFocusPrefix, trainingRounds, testRounds)
        
        # Obtain the median intrinsics and distortion parameters
        temp = np.argsort(focusList[0])
        medianInd = temp[len(temp) // 2]
        defaultIntrinsics = intrinsicsList[0][medianInd]
        dist = distortionList[0][medianInd]
        medFocus = focusList[0][medianInd]
    
    else:
        # Load the appropriate intrinsics, distCoeffs
        defaultIntrinsics = np.load(path + '/' + subfolderPrefixes + str(AFround) + '/' + cameraName + '_intrinsics.npy')
        
        dist = np.load(path + '/' + subfolderPrefixes + str(AFround) + '/' + cameraName + '_distCoeffs.npy')
    
    foldername = 'LbeamTest'
    filenames = glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.jpg') + \
                glob.glob('C:/Users/aofeldman/Desktop/' + str(foldername) + '/*.tif')
    
    # Aaron added in reading in as grayscale
    image_list = [cv2.imread(img, 0) for img in filenames]
    
    print("Number of Images in Folder: " + str(len(image_list)))
    
    estimatedRolls = []
    estimatedDistances = []
    estimatedTranslations = []
    deltaMlist = []
    originShiftList = []
    
    for i, image in enumerate(image_list):    
        print('On Image: ' + str(i))
        bestChorners, bestChids, _, _, chosenSplit = findBoards(image, aruco_dict, board, False, False)
        
        numImagesSkipped = 0
        
        if chosenSplit != -1:
            poses = []
            Ms = []
        
            for j in range(2):
                chorners = bestChorners[j]
                chids = bestChids[j]
                
                rvec, tvec, qHatList = altThinLensPose(pixel2world, f, defaultIntrinsics, dist, chorners, chids, board)
                Ms.append(tvec[2] / qHatList[-1])
                
                # Could instead/also use the median intrinsics
                # rvec, tvec = predictPoseFromIntrinsics(defaultIntrinsics, dist, chorners, chids, board)
                
                poses.append([rvec, tvec])
            
            rvec, tvec, degRoll, distance = computeRelPose(*poses)
            
            deltaMlist.append(Ms[1] - Ms[0])
            originShiftList.append(poses[1][1] - poses[0][1])
            
            #poses, rvec, tvec, degRoll, distance = ThinLensBoardDetect(bestChorners, bestChids, pixel2world, f, defaultIntrinsics, dist, board)
            
            estimatedRolls.append(degRoll)
            
            print('Rotating the left board by ' + str(degRoll) + ' deg gives the right board orientation')
            
            estimatedDistances.append(distance)
                
            print('Distance between the left and right boards is ' + str(distance) + ' m')
            
            estimatedTranslations.append(tvec)    
        
        else:
            print('Failed to find boards: skipping image ' + str(imgIndex))
            numImagesSkipped += 1
    
    # Should print using the altThinLensPose (and the default intrinsics from round AFround3 or 8-31)
    # Sample size: 20
    # Mean of rotation (deg): 0.575096362445919
    # Standard Deviation of rotation (deg): 0.33213241708997654
    # Mean of Distance (m): 0.32575182019076865
    # Standard Deviation of Distance (m): 0.0006502066540464371
    
    print('Sample size: ' + str(len(image_list) - numImagesSkipped))
    print('Mean of rotation (deg): ' + str(np.mean(estimatedRolls)))
    print('Standard Deviation of rotation (deg): ' + str(np.std(estimatedRolls)))
    print('Mean of Distance (m): ' + str(np.mean(estimatedDistances)))
    print('Standard Deviation of Distance (m): ' + str(np.std(estimatedDistances)))
    
    # Almost exactly agree which would expect assuming convergence, difference is in the 1e-16
    # M1 = z1 / (f/M1 + f) equivalent to z1 = fM1 + f so z2 - z1 = f(M1 - M2)
    plt.figure()
    plt.title('Comparing f * deltaM to Result')
    imageRange = range(len(image_list))
    plt.scatter(imageRange, f * np.array(deltaMlist), label='Theory')
    plt.scatter(imageRange, np.array(originShiftList)[:,2], label='Measured')
    plt.xlabel('Image Number')
    plt.ylabel('Difference in Z (optical axis) [m]')
    plt.legend()