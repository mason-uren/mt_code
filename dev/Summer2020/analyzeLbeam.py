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
import estimateBeam

if __name__ == '__main__':
    plt.close('all')
    ##########################
    # Charuco Board Consts   #
    ##########################
    
    fiducialSquareLength = boards['NewFiducial']['squareLength']
    fiducialMarkerLength = boards['NewFiducial']['markerLength']
    # In the images, seems like board is actually rotated so that have 12 across 
    # by 8 down 
    fiducialCharucoX = boards['NewFiducial']['charucoX']
    fiducialCharucoY = boards['NewFiducial']['charucoY']
    
        
    squareLength = boards['TV_3']['squareLength']
    markerLength = boards['TV_3']['markerLength']
    charucoX = boards['TV_3']['charucoX']
    charucoY = boards['TV_3']['charucoY']
    
    ##########################
    # Graphng Setup          #
    ##########################
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    
    fiducial = cv2.aruco.CharucoBoard_create(fiducialCharucoX, fiducialCharucoY, fiducialSquareLength, fiducialMarkerLength, aruco_dict)
    
    # Aaron added to verify that board is indeed what are seeing in the images
    #dispBoard = fiducial.draw((500, 500))
    #cv2.imshow('board', dispBoard)
    #cv2.imshow('board', cv2.rotate(dispBoard, cv2.ROTATE_90_CLOCKWISE))
    #cv2.waitKey(0)
    
    board = cv2.aruco.CharucoBoard_create(charucoX, charucoY, squareLength, markerLength, aruco_dict)
    
    
    ##########################
    # Load Images from Path  #
    ##########################    
    # Need to separate the imperx and ximea
    cameraName = 'ximea'
    
    path = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined'
    subfolderPrefixes = 'AFround'
    relDistName = cameraName + '_distCoeffs.npy'
    relIntrinsicsName = cameraName + '_intrinsics.npy'
    relFocusPrefix = 'AF'
    relLbeam = 'L-Beam'
    numRounds = 19
    chosenRound = 12
    runs = 3
    
    #prefix = 'C:/Users/aofeldman/Desktop/testCollection7-21Refined/AFround12/'
    foldername = path + '/' + subfolderPrefixes + str(chosenRound) + '/'
    
    meanList = []
    lowerList = []
    upperList = []
    intrinsicsOverallList = []
    distOverallList = []
    for run in range(runs):
        print('On run: ' + str(run))
        # Load the calibration images
        filenames = glob.glob(foldername + '*.jpg') + \
                    glob.glob(foldername + '*.tif')
        
        calibImages = [cv2.imread(img, 0) for img in filenames]
        # Slightly blur the calibration images
        calibImages = [cv2.blur(img, (3,3)) for img in calibImages]
        
        # Permute the order
        calibImages = np.random.permutation(calibImages)

        print(calibImages[0].shape)
        
        print("Number of Images for Calibration: " + str(len(calibImages)))
        
        # Load the L-Beam images
        #filenames = glob.glob(foldername + 'L-Beam/' + '*.jpg') + \
        #            glob.glob(foldername + 'L-Beam/' + '*.tif')
        filenames = glob.glob('C:/Users/aofeldman/Desktop/LbeamTest/' + '*.jpg') + \
                    glob.glob('C:/Users/aofeldman/Desktop/LbeamTest/' + '*.tif')
        
        testImages = [cv2.imread(img, 0) for img in filenames]
        
        print("Number of images for testing: " + str(len(testImages)))
        
        means, lowerErr, upperErr, intrinsicsList, distList = estimateBeam.LBeamLearningCurve(calibImages, testImages, 5, aruco_dict, fiducial, board)
        meanList.append(means)
        lowerList.append(lowerErr)
        upperList.append(upperErr)
        intrinsicsOverallList.append(intrinsicsList)
        distOverallList.append(distList)
    